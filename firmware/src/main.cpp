// exxeta-arm firmware: the ESP32 is a "smart I/O" device (PROJECT_SPEC 3 and 7).
//
// It does NO kinematics and NO planning. It
//   1. accepts joint targets from ONE PC over USB serial or Wi-Fi/UDP (same protocol),
//   2. moves six hobby servos smoothly towards them (speed/acceleration limited),
//   3. enforces joint limits and speed limits on its own (second safety layer),
//   4. holds position and latches FLT_LINK_LOST if the PC goes quiet (watchdog),
//   5. reports state, battery and optional pot readings 50 times a second,
//   6. keeps its configuration (calibration, limits, Wi-Fi, name, token) in flash.
//
// File layout: configuration -> servo output -> sensors -> LED -> protocol handling
// -> transports (serial, UDP) -> Wi-Fi provisioning -> debug CLI -> setup/loop.
// Pure logic (protocol, motion) lives in lib/armcore so it can be unit-tested on a PC.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "arm_defaults.h"
#include "motion.h"
#include "pinmap.h"
#include "protocol.h"

using namespace proto;

static const uint8_t FW_VERSION[3] = {0, 1, 0};
static const uint16_t UDP_PORT = 47800;
static const uint32_t CONTROL_PERIOD_US = 10000;   // 100 Hz motion update (PROJECT_SPEC 5.2)
static const uint32_t ATTACH_INTERVAL_MS = 150;    // staged attach: one servo at a time
static const uint32_t SESSION_TIMEOUT_MS = 3000;   // after this silence another PC may take control
static const uint16_t DEADBAND_US = 2;             // ignore pulse changes smaller than this (less jitter and heat)

// ===========================================================================
// Configuration (stored in flash / NVS)
// ===========================================================================
struct Config {
  uint32_t magic;
  ServoCal cal[6];
  float minDeg[6], maxDeg[6], vmax[6], amax[6];
  uint16_t potZero[4];
  float potPerDeg[4];
  uint8_t potEnabled[4];
  char name[24];
  uint32_t token;                 // 0 = no token required
  char ssid[3][33];
  char pass[3][65];
  uint16_t watchdogMs;
  uint8_t stateHz;
  float rest[6];
};
static const uint32_t CONFIG_MAGIC = 0xE77A0004;   // change when the struct layout changes
static Config cfg;
static Preferences prefs;

static void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CONFIG_MAGIC;
  for (int i = 0; i < 6; i++) {
    cfg.cal[i] = ServoCal();               // 500..2500 us, 11.11 us/deg (180 deg servo)
    cfg.cal[i].zeroUs = DEF_ZERO_US[i];    // servo centre = middle of the joint range
    cfg.minDeg[i] = DEF_MIN_DEG[i];
    cfg.maxDeg[i] = DEF_MAX_DEG[i];
    cfg.vmax[i] = DEF_MAX_SPEED[i];
    cfg.amax[i] = DEF_MAX_ACCEL[i];
    cfg.rest[i] = DEF_REST_POSE[i];
  }
  for (int i = 0; i < 4; i++) { cfg.potZero[i] = 2048; cfg.potPerDeg[i] = 4095.0f / 300.0f; cfg.potEnabled[i] = 0; }
  strlcpy(cfg.name, "exxeta-arm", sizeof(cfg.name));
  cfg.watchdogMs = 500;
  cfg.stateHz = 50;
}

static void configLoad() {
  prefs.begin("exxeta", false);
  if (prefs.getBytesLength("cfg") != sizeof(cfg) || prefs.getBytes("cfg", &cfg, sizeof(cfg)) != sizeof(cfg) ||
      cfg.magic != CONFIG_MAGIC) {
    configDefaults();
  }
}
static void configSave() { prefs.putBytes("cfg", &cfg, sizeof(cfg)); }

// ===========================================================================
// State
// ===========================================================================
static JointMotion joint[6];
static bool attached[6] = {false};
static uint16_t lastUs[6] = {0};
static uint16_t rawUs[6] = {0};         // calibration override: raw pulse for one servo
static uint8_t mode = MODE_DISABLED;
static uint16_t faults = 0;
static int attachNext = -1;             // >= 0 while the staged attach is running
static uint32_t attachAtMs = 0;
static bool parkingForBattery = false;

enum LinkKind : uint8_t { LINK_NONE, LINK_SERIAL, LINK_UDP };
struct Session {
  bool active = false;
  LinkKind link = LINK_NONE;
  uint32_t clientId = 0;
  IPAddress ip;
  uint16_t port = 0;
  uint32_t lastRxMs = 0;
} session;
static uint16_t rxOk = 0, rxBad = 0, lastSeq = 0, txSeq = 0;

static float battV = 0;                 // filtered pack voltage
static float potDeg[4] = {0};
static uint8_t potValid = 0;
static bool apMode = false;

static WiFiUDP udp;
static WebServer portal(80);
static DNSServer dns;

// ===========================================================================
// Servo output (LEDC: 50 Hz, 16 bit)
// ===========================================================================
static void servoWrite(int i, uint16_t us) {
  if (!attached[i]) {
    ledcSetup(i, 50, 16);
    ledcAttachPin(PIN_SERVO[i], i);
    attached[i] = true;
    lastUs[i] = 0;
  }
  if (abs((int)us - (int)lastUs[i]) < DEADBAND_US) return;
  lastUs[i] = us;
  ledcWrite(i, (uint32_t)us * 65536UL / 20000UL);
}

static void servoDetachAll() {
  for (int i = 0; i < 6; i++) {
    if (attached[i]) { ledcDetachPin(PIN_SERVO[i]); pinMode(PIN_SERVO[i], INPUT); }
    attached[i] = false;
    rawUs[i] = 0;
  }
  attachNext = -1;
}

static void applyLimitsFromConfig() {
  for (int i = 0; i < 6; i++) {
    joint[i].minDeg = cfg.minDeg[i];
    joint[i].maxDeg = cfg.maxDeg[i];
    joint[i].vmax = cfg.vmax[i];
    joint[i].amax = cfg.amax[i];
  }
}

// ===========================================================================
// Sensors: battery and optional pots (all on ADC1)
// ===========================================================================
static void sensorsRead(float dt) {
  float v = analogReadMilliVolts(PIN_BATTERY) / 1000.0f * BATT_DIVIDER;
  float a = dt / (BATT_FILTER_TAU_S + dt);      // first-order low-pass: current spikes must not trigger a park
  battV = (battV == 0) ? v : battV + a * (v - battV);

  potValid = 0;
  for (int i = 0; i < 4; i++) {
    if (!cfg.potEnabled[i]) continue;
    int raw = analogRead(PIN_POT[i]);
    float deg = (raw - (int)cfg.potZero[i]) / cfg.potPerDeg[i];
    potDeg[i] += 0.2f * (deg - potDeg[i]);
    if (raw > 60 && raw < 4030) potValid |= (1 << i);   // a reading at the rail means broken wire or end of track
  }

  // battery state with hysteresis
  uint16_t f = faults & ~(FLT_SERVO_POWER_OFF | FLT_CHARGER_PRESENT);
  if (battV < BATT_ABSENT_V) f |= FLT_SERVO_POWER_OFF;
  if (battV > BATT_CHARGER_V) f |= FLT_CHARGER_PRESENT;
  if (battV >= BATT_ABSENT_V) {
    if (battV < BATT_WARN_V) f |= FLT_BATTERY_WARN;
    else if (battV > BATT_WARN_V + BATT_HYSTERESIS_V) f &= ~FLT_BATTERY_WARN;
    if (battV < BATT_CRITICAL_V) f |= FLT_BATTERY_CRITICAL;
    else if (battV > BATT_CRITICAL_V + BATT_HYSTERESIS_V) f &= ~FLT_BATTERY_CRITICAL;
  } else {
    f &= ~(FLT_BATTERY_WARN | FLT_BATTERY_CRITICAL);
  }
  faults = f;
}

static uint8_t batteryPercent() {
  // 2S Li-ion, charged to 4.10 V per cell (ADR-0003), 0 % at the critical threshold
  static const float V[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f};
  static const float P[] = {0, 10, 25, 45, 60, 75, 88, 100};
  float c = battV / 2;
  if (c <= V[0]) return 0;
  for (int i = 1; i < 8; i++)
    if (c < V[i]) return (uint8_t)(P[i - 1] + (P[i] - P[i - 1]) * (c - V[i - 1]) / (V[i] - V[i - 1]));
  return 100;
}

// ===========================================================================
// RGB status LED (colour codes: docs/firmware/led_codes.md)
// ===========================================================================
static void ledUpdate() {
  uint32_t t = millis();
  bool slow = (t / 500) % 2, fast = (t / 150) % 2;
  uint8_t r = 0, g = 0, b = 0;
  if (apMode) { r = slow ? 24 : 0; b = slow ? 24 : 0; }                          // magenta blink: Wi-Fi setup hotspot
  else if (faults & (FLT_LINK_LOST | FLT_STOP_LATCHED | FLT_BATTERY_CRITICAL)) { r = fast ? 40 : 4; }   // red blink: fault
  else if (faults & FLT_BATTERY_WARN) { r = slow ? 30 : 0; g = slow ? 18 : 0; }      // yellow blink: battery low
  else if (!session.active) { b = slow ? 24 : 2; }                               // blue blink: waiting for PC
  else if (mode == MODE_DISABLED) { g = 6; b = 6; }                                   // dim cyan: connected, servos off
  else if (mode == MODE_MOVING) { g = 30; b = 10; }                                   // bright green: moving
  else { g = 16; }                                                               // green: enabled, idle
  neopixelWrite(PIN_RGB_LED, r, g, b);
}

// ===========================================================================
// Sending
// ===========================================================================
static void sendFrame(uint8_t type, uint16_t seq, const uint8_t* payload, size_t len, LinkKind link, IPAddress ip,
                      uint16_t port) {
  uint8_t raw[MAX_FRAME], enc[MAX_FRAME + 8];
  size_t n = encodeFrame(type, seq, payload, len, raw);
  if (link == LINK_SERIAL) {
    size_t m = cobsEncode(raw, n, enc + 1);
    enc[0] = 0;
    enc[m + 1] = 0;
    Serial.write(enc, m + 2);
  } else if (link == LINK_UDP) {
    udp.beginPacket(ip, port);
    udp.write(raw, n);
    udp.endPacket();
  }
}
static void sendToSession(uint8_t type, const uint8_t* payload, size_t len) {
  if (session.active) sendFrame(type, txSeq++, payload, len, session.link, session.ip, session.port);
}
static void sendAck(const Frame& f, uint8_t status, LinkKind link, IPAddress ip, uint16_t port) {
  uint8_t p[4];
  put16(p, f.seq);
  p[2] = f.type;
  p[3] = status;
  sendFrame(MSG_ACK, txSeq++, p, 4, link, ip, port);
}
static void logMsg(const char* text) {
  if (session.active && session.link == LINK_SERIAL) {
    uint8_t p[100];
    size_t n = strnlen(text, 98);
    p[0] = 1;
    memcpy(p + 1, text, n);
    sendToSession(MSG_LOG, p, n + 1);
  } else {
    Serial.print("# ");
    Serial.println(text);
  }
}

static void sendState() {
  State s;
  s.mode = mode;
  s.faults = faults;
  for (int i = 0; i < 6; i++) { s.targetDeg[i] = joint[i].target; s.positionDeg[i] = joint[i].pos; }
  for (int i = 0; i < 4; i++) s.potDeg[i] = potDeg[i];
  s.potValid = potValid;
  s.batteryMv = (uint16_t)(battV * 1000);
  s.batteryPct = batteryPercent();
  s.uptimeMs = millis();
  s.rxOk = rxOk; s.rxBad = rxBad; s.lastSeq = lastSeq;
  uint8_t p[STATE_SIZE];
  sendToSession(MSG_STATE, p, packState(s, p));
}

// ===========================================================================
// Commands
// ===========================================================================
static bool motionAllowed() {
  return (mode == MODE_IDLE || mode == MODE_MOVING) && !(faults & (FLT_LINK_LOST | FLT_STOP_LATCHED | FLT_BATTERY_CRITICAL | FLT_SERVO_POWER_OFF));
}

// PROJECT_SPEC 7.3: without pots the arm is ASSUMED to be parked in the rest pose.
static uint8_t cmdEnable(bool usePots) {
  if (faults & (FLT_SERVO_POWER_OFF | FLT_BATTERY_CRITICAL | FLT_LINK_LOST | FLT_STOP_LATCHED)) return ST_REFUSED;
  if (mode != MODE_DISABLED) return ST_OK;
  for (int i = 0; i < 6; i++) {
    float start = cfg.rest[i];
    if (usePots && i < 4 && (potValid & (1 << i))) start = potDeg[i];   // start exactly where the joint really is
    joint[i].reset(constrain(start, cfg.minDeg[i], cfg.maxDeg[i]));
    rawUs[i] = 0;
  }
  attachNext = 0;
  attachAtMs = millis();
  mode = MODE_IDLE;
  return ST_OK;
}
static void cmdDisable() {
  servoDetachAll();
  mode = MODE_DISABLED;
  parkingForBattery = false;
}
static void cmdStop() {
  for (int i = 0; i < 6; i++) joint[i].stop();
  if (mode != MODE_DISABLED) mode = MODE_STOPPED;
  faults |= FLT_STOP_LATCHED;
}
static void cmdClear() {
  faults &= ~(FLT_STOP_LATCHED | FLT_LINK_LOST | FLT_LIMIT_CLAMPED);
  for (int i = 0; i < 6; i++) joint[i].target = joint[i].pos;
  if (mode == MODE_STOPPED || mode == MODE_FAULT) mode = MODE_IDLE;
}

static uint8_t cmdSetConfig(const uint8_t* p, size_t n) {
  if (n < 2) return ST_BAD_ARG;
  uint8_t key = p[0], idx = p[1];
  const uint8_t* d = p + 2;
  size_t len = n - 2;
  switch (key) {
    case CFG_SERVO_CAL:
      if (idx >= 6 || len < 8) return ST_BAD_ARG;
      cfg.cal[idx].minUs = get16(d); cfg.cal[idx].maxUs = get16(d + 2); cfg.cal[idx].zeroUs = get16(d + 4);
      cfg.cal[idx].usPerDeg = get16s(d + 6) / 100.0f;
      return ST_OK;
    case CFG_JOINT_LIMITS:
      if (idx >= 6 || len < 8) return ST_BAD_ARG;
      cfg.minDeg[idx] = get16s(d) / 100.0f; cfg.maxDeg[idx] = get16s(d + 2) / 100.0f;
      cfg.vmax[idx] = get16(d + 4) / 10.0f; cfg.amax[idx] = get16(d + 6);
      applyLimitsFromConfig();
      return ST_OK;
    case CFG_POT_CAL:
      if (idx >= 4 || len < 5) return ST_BAD_ARG;
      cfg.potZero[idx] = get16(d); cfg.potPerDeg[idx] = get16s(d + 2) / 100.0f; cfg.potEnabled[idx] = d[4];
      if (cfg.potPerDeg[idx] == 0) cfg.potPerDeg[idx] = 1;
      return ST_OK;
    case CFG_DEVICE_NAME:
      if (len == 0 || len >= sizeof(cfg.name)) return ST_BAD_ARG;
      memcpy(cfg.name, d, len); cfg.name[len] = 0;
      return ST_OK;
    case CFG_AUTH_TOKEN:
      if (len < 4) return ST_BAD_ARG;
      cfg.token = get32(d);
      return ST_OK;
    case CFG_WIFI: {
      if (idx >= 3 || len < 2) return ST_BAD_ARG;
      uint8_t sl = d[0];
      if (sl > 32 || len < (size_t)sl + 2) return ST_BAD_ARG;
      uint8_t pl = d[1 + sl];
      if (pl > 64 || len < (size_t)sl + pl + 2) return ST_BAD_ARG;
      memcpy(cfg.ssid[idx], d + 1, sl); cfg.ssid[idx][sl] = 0;
      memcpy(cfg.pass[idx], d + 2 + sl, pl); cfg.pass[idx][pl] = 0;
      return ST_OK;
    }
    case CFG_WATCHDOG_MS:
      if (len < 2 || get16(d) < 100) return ST_BAD_ARG;
      cfg.watchdogMs = get16(d);
      return ST_OK;
    case CFG_STATE_RATE_HZ:
      if (len < 1 || d[0] < 1 || d[0] > 100) return ST_BAD_ARG;
      cfg.stateHz = d[0];
      return ST_OK;
    case CFG_REST_POSE:
      if (len < 12) return ST_BAD_ARG;
      for (int i = 0; i < 6; i++) cfg.rest[i] = get16s(d + 2 * i) / 100.0f;
      return ST_OK;
  }
  return ST_BAD_ARG;
}

static size_t cmdGetConfig(uint8_t key, uint8_t idx, uint8_t* out) {
  out[0] = key; out[1] = idx;
  uint8_t* d = out + 2;
  switch (key) {
    case CFG_SERVO_CAL:
      if (idx >= 6) return 0;
      put16(d, cfg.cal[idx].minUs); put16(d + 2, cfg.cal[idx].maxUs); put16(d + 4, cfg.cal[idx].zeroUs);
      put16(d + 6, (uint16_t)(int16_t)lroundf(cfg.cal[idx].usPerDeg * 100));
      return 10;
    case CFG_JOINT_LIMITS:
      if (idx >= 6) return 0;
      put16(d, (uint16_t)toCdeg(cfg.minDeg[idx])); put16(d + 2, (uint16_t)toCdeg(cfg.maxDeg[idx]));
      put16(d + 4, (uint16_t)(cfg.vmax[idx] * 10)); put16(d + 6, (uint16_t)cfg.amax[idx]);
      return 10;
    case CFG_POT_CAL:
      if (idx >= 4) return 0;
      put16(d, cfg.potZero[idx]); put16(d + 2, (uint16_t)(int16_t)lroundf(cfg.potPerDeg[idx] * 100)); d[4] = cfg.potEnabled[idx];
      return 7;
    case CFG_DEVICE_NAME: { size_t n = strlen(cfg.name); memcpy(d, cfg.name, n); return 2 + n; }
    case CFG_WATCHDOG_MS: put16(d, cfg.watchdogMs); return 4;
    case CFG_STATE_RATE_HZ: d[0] = cfg.stateHz; return 3;
    case CFG_REST_POSE: for (int i = 0; i < 6; i++) put16(d + 2 * i, (uint16_t)toCdeg(cfg.rest[i])); return 14;
  }
  return 0;   // token and Wi-Fi passwords are write-only
}

static bool fromOwner(LinkKind link, IPAddress ip, uint16_t port) {
  if (!session.active || session.link != link) return false;
  return link == LINK_SERIAL || (session.ip == ip && session.port == port);
}

static void handleFrame(const Frame& f, LinkKind link, IPAddress ip, uint16_t port) {
  uint32_t now = millis();
  if (f.type == MSG_HELLO) {
    if (f.len < 8) return;
    uint32_t token = get32(f.payload), client = get32(f.payload + 4);
    uint8_t status = ST_OK;
    bool stale = session.active && (now - session.lastRxMs > SESSION_TIMEOUT_MS);
    if (cfg.token != 0 && token != cfg.token) status = ST_BAD_TOKEN;
    else if (session.active && !stale && !(session.clientId == client && session.link == link)) status = ST_BUSY;  // single controller
    if (status == ST_OK) {
      session.active = true; session.link = link; session.clientId = client;
      session.ip = ip; session.port = port; session.lastRxMs = now;
    }
    uint8_t p[40];
    size_t nl = strlen(cfg.name);
    p[0] = status; p[1] = VERSION; p[2] = FW_VERSION[0]; p[3] = FW_VERSION[1]; p[4] = FW_VERSION[2];
    p[5] = N_JOINTS; p[6] = (uint8_t)nl;
    memcpy(p + 7, cfg.name, nl);
    sendFrame(MSG_HELLO_ACK, txSeq++, p, 7 + nl, link, ip, port);
    return;
  }
  if (!fromOwner(link, ip, port)) return;       // only the controlling PC is listened to
  session.lastRxMs = now;
  lastSeq = f.seq;

  switch (f.type) {
    case MSG_HEARTBEAT: break;
    case MSG_ENABLE: sendAck(f, cmdEnable(f.len >= 1 && (f.payload[0] & 1)), link, ip, port); break;
    case MSG_DISABLE: cmdDisable(); sendAck(f, ST_OK, link, ip, port); break;
    case MSG_SET_JOINT_TARGETS:
      if (f.len >= 12 && motionAllowed() && !parkingForBattery) {
        bool clampedAny = false;
        for (int i = 0; i < 6; i++) {
          float cap = (f.len >= 24) ? get16(f.payload + 12 + 2 * i) / 10.0f : 0;
          joint[i].setTarget(get16s(f.payload + 2 * i) / 100.0f, cap);
          clampedAny |= joint[i].clamped;
        }
        if (clampedAny) faults |= FLT_LIMIT_CLAMPED; else faults &= ~FLT_LIMIT_CLAMPED;
      }
      break;                                    // streamed: no MSG_ACK, the newest target simply wins
    case MSG_STOP: cmdStop(); sendAck(f, ST_OK, link, ip, port); break;
    case MSG_CLEAR: cmdClear(); sendAck(f, ST_OK, link, ip, port); break;
    case MSG_RELEASE: sendAck(f, ST_OK, link, ip, port); session.active = false; break;
    case MSG_SET_CONFIG: sendAck(f, cmdSetConfig(f.payload, f.len), link, ip, port); break;
    case MSG_GET_CONFIG: {
      uint8_t out[80];
      size_t n = (f.len >= 2) ? cmdGetConfig(f.payload[0], f.payload[1], out) : 0;
      if (n) sendFrame(MSG_CONFIG, f.seq, out, n, link, ip, port);
      else sendAck(f, ST_BAD_ARG, link, ip, port);
      break;
    }
    case MSG_CALIBRATION: {
      uint8_t st = ST_BAD_ARG;
      if (f.len >= 1) {
        if (f.payload[0] == CAL_JOG_PULSE && f.len >= 4 && f.payload[1] < 6) {
          // raw pulse on ONE servo, for finding the calibration points. Not while a fault is latched.
          if (faults & (FLT_SERVO_POWER_OFF | FLT_STOP_LATCHED | FLT_LINK_LOST)) st = ST_REFUSED;
          else { rawUs[f.payload[1]] = constrain(get16(f.payload + 2), 400, 2600); st = ST_OK; }
        } else if (f.payload[0] == CAL_SAVE) { configSave(); st = ST_OK; }
        else if (f.payload[0] == CAL_FACTORY_RESET) { configDefaults(); applyLimitsFromConfig(); configSave(); st = ST_OK; }
      }
      sendAck(f, st, link, ip, port);
      break;
    }
  }
}

// ===========================================================================
// Transports
// ===========================================================================
static void cliExecute(char* line);

// Serial: 0x00 COBS(frame) 0x00 = binary frame; a text line ending in '\n' = debug CLI.
static void serialPoll() {
  static uint8_t buf[MAX_FRAME + 8];
  static size_t n = 0;
  static bool inFrame = false;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (b == 0) {
      if (inFrame && n > 0) {
        uint8_t raw[MAX_FRAME + 8];
        size_t m = cobsDecode(buf, n, raw);
        Frame f;
        if (m && decodeFrame(raw, m, f)) { rxOk++; handleFrame(f, LINK_SERIAL, IPAddress(), 0); }
        else rxBad++;
        inFrame = false;
      } else {
        inFrame = true;
      }
      n = 0;
    } else if (inFrame) {
      if (n < sizeof(buf)) buf[n++] = b; else { n = 0; inFrame = false; rxBad++; }
    } else if (b == '\n') {
      buf[n] = 0;
      if (n > 0 && buf[n - 1] == '\r') buf[n - 1] = 0;
      if (n > 0) cliExecute((char*)buf);
      n = 0;
    } else if (n < 120) {
      buf[n++] = b;
    }
  }
}

static void udpPoll() {
  if (WiFi.status() != WL_CONNECTED) return;
  int len;
  while ((len = udp.parsePacket()) > 0) {
    uint8_t raw[MAX_FRAME + 8];
    int n = udp.read(raw, sizeof(raw));
    Frame f;
    if (n > 0 && decodeFrame(raw, n, f)) { rxOk++; handleFrame(f, LINK_UDP, udp.remoteIP(), udp.remotePort()); }
    else rxBad++;
  }
}

// ===========================================================================
// Wi-Fi: saved networks first, else own hotspot with a setup page (D9)
// ===========================================================================
static String chipSuffix() {
  char s[5];
  snprintf(s, sizeof(s), "%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  return String(s);
}

static const char PORTAL_PAGE[] PROGMEM =
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>exxeta-arm Wi-Fi setup</title>"
    "<body style='font-family:sans-serif;max-width:26em;margin:2em auto;padding:0 1em'>"
    "<h2>exxeta-arm Wi-Fi setup</h2><p>Enter the network the arm should join. It restarts afterwards.</p>"
    "<form method=post action=/save><p>Network name (SSID)<br><input name=s maxlength=32 style='width:100%;font-size:1.2em'>"
    "<p>Password<br><input name=p type=password maxlength=64 style='width:100%;font-size:1.2em'>"
    "<p><button style='font-size:1.2em;padding:.5em 1em'>Save and restart</button></form>"
    "<p><small>This page only sets up Wi-Fi. The arm cannot be controlled from here.</small></body>";

static void portalStart() {
  apMode = true;
  String ap = "exxeta-arm-" + chipSuffix();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap.c_str());
  dns.start(53, "*", WiFi.softAPIP());        // captive portal: every name resolves to us
  portal.on("/save", HTTP_POST, []() {
    String s = portal.arg("s"), p = portal.arg("p");
    if (s.length() > 0 && s.length() <= 32 && p.length() <= 64) {
      memmove(cfg.ssid[1], cfg.ssid[0], sizeof(cfg.ssid[0]) * 2);   // keep older networks in slots 1 and 2
      memmove(cfg.pass[1], cfg.pass[0], sizeof(cfg.pass[0]) * 2);
      strlcpy(cfg.ssid[0], s.c_str(), sizeof(cfg.ssid[0]));
      strlcpy(cfg.pass[0], p.c_str(), sizeof(cfg.pass[0]));
      configSave();
      portal.send(200, "text/html", "<body style='font-family:sans-serif'><h2>Saved. Restarting...</h2>");
      delay(800);
      ESP.restart();
    } else {
      portal.send(400, "text/plain", "Invalid input");
    }
  });
  portal.onNotFound([]() { portal.send_P(200, "text/html", PORTAL_PAGE); });
  portal.begin();
  Serial.printf("# Wi-Fi setup hotspot '%s' started, open http://192.168.4.1\n", ap.c_str());
}

static void wifiStart() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg.name);
  for (int slot = 0; slot < 3; slot++) {
    if (cfg.ssid[slot][0] == 0) continue;
    Serial.printf("# Wi-Fi: trying '%s'\n", cfg.ssid[slot]);
    WiFi.begin(cfg.ssid[slot], cfg.pass[slot]);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(100); ledUpdate(); }
    if (WiFi.status() == WL_CONNECTED) break;
    WiFi.disconnect();
  }
  if (WiFi.status() != WL_CONNECTED) { portalStart(); return; }
  WiFi.setSleep(false);                         // lower latency for the 50 Hz command stream
  udp.begin(UDP_PORT);
  if (MDNS.begin(cfg.name)) MDNS.addService("exxeta-arm", "udp", UDP_PORT);
  ArduinoOTA.setHostname(cfg.name);
  ArduinoOTA.onStart([]() { cmdDisable(); });   // never update with live servos
  ArduinoOTA.begin();
  Serial.printf("# Wi-Fi connected, IP %s, UDP port %u, mDNS %s.local\n", WiFi.localIP().toString().c_str(), UDP_PORT, cfg.name);
}

// ===========================================================================
// Debug CLI (text lines over USB serial; see docs/firmware/cli.md)
// ===========================================================================
static void cliExecute(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " ");
  char* a2 = strtok(nullptr, "");
  if (!strcmp(cmd, "help")) {
    Serial.println("# commands: status | enable | disable | stop | clear | move <joint 1-6> <deg> | pulse <joint 1-6> <us>");
    Serial.println("#           wifi <ssid> <password> | name <name> | token <number> | save | reset | reboot");
  } else if (!strcmp(cmd, "status")) {
    Serial.printf("# %s fw %u.%u.%u mode %u faults 0x%04X battery %.2f V (%u %%) session %d ip %s\n", cfg.name, FW_VERSION[0],
                  FW_VERSION[1], FW_VERSION[2], mode, faults, battV, batteryPercent(), session.active,
                  apMode ? "AP 192.168.4.1" : WiFi.localIP().toString().c_str());
    for (int i = 0; i < 6; i++)
      Serial.printf("#  J%d pos %7.2f target %7.2f limits %.0f..%.0f pulse %u\n", i + 1, joint[i].pos, joint[i].target,
                    cfg.minDeg[i], cfg.maxDeg[i], lastUs[i]);
  } else if (!strcmp(cmd, "enable")) { Serial.printf("# enable -> %u\n", cmdEnable(false)); }
  else if (!strcmp(cmd, "disable")) { cmdDisable(); Serial.println("# disabled"); }
  else if (!strcmp(cmd, "stop")) { cmdStop(); Serial.println("# stopped (latched; use 'clear')"); }
  else if (!strcmp(cmd, "clear")) { cmdClear(); Serial.println("# cleared"); }
  else if (!strcmp(cmd, "move") && a1 && a2) {
    int j = atoi(a1) - 1;
    if (j >= 0 && j < 6 && motionAllowed()) { joint[j].setTarget(atof(a2)); Serial.println("# ok"); }
    else Serial.println("# refused (enable first, joint 1-6)");
  } else if (!strcmp(cmd, "pulse") && a1 && a2) {
    int j = atoi(a1) - 1;
    if (j >= 0 && j < 6) { rawUs[j] = constrain(atoi(a2), 400, 2600); Serial.println("# ok (raw pulse; 'disable' releases)"); }
  } else if (!strcmp(cmd, "wifi") && a1 && a2) {
    strlcpy(cfg.ssid[0], a1, sizeof(cfg.ssid[0]));
    strlcpy(cfg.pass[0], a2, sizeof(cfg.pass[0]));
    configSave();
    Serial.println("# saved; 'reboot' to connect");
  } else if (!strcmp(cmd, "name") && a1) { strlcpy(cfg.name, a1, sizeof(cfg.name)); configSave(); Serial.println("# saved"); }
  else if (!strcmp(cmd, "token") && a1) { cfg.token = strtoul(a1, nullptr, 0); configSave(); Serial.println("# saved"); }
  else if (!strcmp(cmd, "save")) { configSave(); Serial.println("# saved"); }
  else if (!strcmp(cmd, "reset")) { configDefaults(); applyLimitsFromConfig(); configSave(); Serial.println("# factory defaults restored"); }
  else if (!strcmp(cmd, "reboot")) { cmdDisable(); delay(100); ESP.restart(); }
  else Serial.println("# unknown command, try 'help'");
}

// ===========================================================================
// Control tick (100 Hz)
// ===========================================================================
static void controlTick(float dt) {
  uint32_t now = millis();

  // link watchdog: hold position (never detach: the arm would fall) and latch FLT_LINK_LOST
  if (session.active && now - session.lastRxMs > cfg.watchdogMs) {
    if (mode != MODE_DISABLED && !(faults & FLT_LINK_LOST)) {
      for (int i = 0; i < 6; i++) joint[i].stop();
      faults |= FLT_LINK_LOST;
    }
    if (now - session.lastRxMs > SESSION_TIMEOUT_MS) session.active = false;
  }

  // critical battery: park in the rest pose once, then refuse motion
  if ((faults & FLT_BATTERY_CRITICAL) && mode != MODE_DISABLED && !parkingForBattery) {
    parkingForBattery = true;
    for (int i = 0; i < 6; i++) joint[i].setTarget(cfg.rest[i], 30);
  }
  // main switch turned off while enabled: nothing to hold any more
  if ((faults & FLT_SERVO_POWER_OFF) && mode != MODE_DISABLED) cmdDisable();

  if (mode == MODE_DISABLED) {
    for (int i = 0; i < 6; i++)
      if (rawUs[i]) servoWrite(i, rawUs[i]);   // calibration jog of a single servo
    return;
  }

  bool any = false;
  for (int i = 0; i < 6; i++) {
    joint[i].step(dt);
    any |= joint[i].moving();
  }
  if (mode == MODE_IDLE || mode == MODE_MOVING) mode = any ? MODE_MOVING : MODE_IDLE;

  // staged attach: start PWM on one servo at a time to limit inrush current
  if (attachNext >= 0 && now >= attachAtMs) {
    servoWrite(attachNext, cfg.cal[attachNext].pulse(joint[attachNext].pos));
    attachAtMs = now + ATTACH_INTERVAL_MS;
    if (++attachNext >= 6) attachNext = -1;
  }
  for (int i = 0; i < 6; i++)
    if (attached[i]) servoWrite(i, rawUs[i] ? rawUs[i] : cfg.cal[i].pulse(joint[i].pos));
}

// ===========================================================================
void setup() {
  for (int i = 0; i < 6; i++) pinMode(PIN_SERVO[i], INPUT);   // PROJECT_SPEC 7.3: no PWM at boot
  Serial.begin(115200);
  neopixelWrite(PIN_RGB_LED, 12, 12, 12);                     // white: booting
  analogSetAttenuation(ADC_11db);
  configLoad();
  applyLimitsFromConfig();
  for (int i = 0; i < 6; i++) joint[i].reset(cfg.rest[i]);
  Serial.printf("\n# exxeta-arm firmware %u.%u.%u, protocol %u. Type 'help'.\n", FW_VERSION[0], FW_VERSION[1], FW_VERSION[2], VERSION);
  wifiStart();
}

void loop() {
  static uint32_t lastTick = micros(), lastState = 0, lastSlow = 0;
  serialPoll();
  if (apMode) { dns.processNextRequest(); portal.handleClient(); }
  else { udpPoll(); if (mode == MODE_DISABLED) ArduinoOTA.handle(); }

  uint32_t nowUs = micros();
  if (nowUs - lastTick >= CONTROL_PERIOD_US) {
    float dt = (nowUs - lastTick) * 1e-6f;
    lastTick = nowUs;
    if (dt > 0.05f) dt = 0.05f;
    controlTick(dt);
  }
  uint32_t now = millis();
  if (now - lastSlow >= 20) {                 // sensors and LED at 50 Hz
    sensorsRead((now - lastSlow) * 0.001f);
    lastSlow = now;
    ledUpdate();
  }
  if (session.active && now - lastState >= 1000u / cfg.stateHz) {
    lastState = now;
    sendState();
  }
}
