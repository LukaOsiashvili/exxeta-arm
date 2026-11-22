// Host unit tests for the pure firmware logic (protocol + motion).
//
// Run on a PC with a C++ compiler:   pio test -e native
// (Not run on the implementer's machine: no host compiler there. See docs/decisions/unverified.md.)
//
// The protocol tests use vectors.h, which is generated from the PYTHON protocol
// implementation. If these tests pass, PC and firmware agree byte for byte.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "motion.h"
#include "protocol.h"
#include "vectors.h"

using namespace proto;

void setUp() {}
void tearDown() {}

void test_vectors_decode() {
  for (size_t i = 0; i < N_VECTORS; i++) {
    const Vector& v = VECTORS[i];
    uint8_t raw[MAX_FRAME + 8];
    size_t n = cobsDecode(v.cobs, v.cobsLen, raw);
    TEST_ASSERT_EQUAL_MESSAGE(v.frameLen, n, v.name);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(v.frame, raw, n, v.name);
    Frame f;
    TEST_ASSERT_TRUE_MESSAGE(decodeFrame(raw, n, f), v.name);
    TEST_ASSERT_EQUAL_MESSAGE(v.type, f.type, v.name);
    TEST_ASSERT_EQUAL_MESSAGE(v.seq, f.seq, v.name);
    TEST_ASSERT_EQUAL_MESSAGE(v.payloadLen, f.len, v.name);
    if (f.len) TEST_ASSERT_EQUAL_MEMORY_MESSAGE(v.payload, f.payload, f.len, v.name);
  }
}

void test_vectors_encode() {
  for (size_t i = 0; i < N_VECTORS; i++) {
    const Vector& v = VECTORS[i];
    uint8_t raw[MAX_FRAME + 8], enc[MAX_FRAME + 8];
    size_t n = encodeFrame(v.type, v.seq, v.payload, v.payloadLen, raw);
    TEST_ASSERT_EQUAL_MESSAGE(v.frameLen, n, v.name);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(v.frame, raw, n, v.name);
    size_t m = cobsEncode(raw, n, enc);
    TEST_ASSERT_EQUAL_MESSAGE(v.cobsLen, m, v.name);
    TEST_ASSERT_EQUAL_MEMORY_MESSAGE(v.cobs, enc, m, v.name);
    for (size_t k = 0; k < m; k++) TEST_ASSERT_NOT_EQUAL(0, enc[k]);   // COBS output never contains zero
  }
}

void test_bad_crc_rejected() {
  uint8_t raw[32];
  size_t n = encodeFrame(MSG_HEARTBEAT, 7, nullptr, 0, raw);
  raw[1] ^= 0x01;
  Frame f;
  TEST_ASSERT_FALSE(decodeFrame(raw, n, f));
}

void test_state_size() {
  State s;
  uint8_t out[STATE_SIZE + 4];
  TEST_ASSERT_EQUAL(STATE_SIZE, packState(s, out));
}

void test_motion_reaches_target_within_limits() {
  JointMotion j;
  j.vmax = 60; j.amax = 180; j.minDeg = -90; j.maxDeg = 90;
  j.reset(0);
  j.setTarget(45);
  float vpeak = 0, prevVel = 0, apeak = 0;
  int steps = 0;
  while (j.moving() && steps < 1000) {
    j.step(0.01f);
    vpeak = fmaxf(vpeak, fabsf(j.vel));
    apeak = fmaxf(apeak, fabsf(j.vel - prevVel) / 0.01f);
    prevVel = j.vel;
    steps++;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 45.0f, j.pos);
  TEST_ASSERT_TRUE(vpeak <= 60.5f);
  TEST_ASSERT_TRUE(apeak <= 181.0f * 2);     // the final snap may add one small step
  TEST_ASSERT_TRUE(steps < 150);             // 45 deg at 60 deg/s takes about 1.1 s
}

void test_motion_clamps_to_limits() {
  JointMotion j;
  j.minDeg = -10; j.maxDeg = 20;
  j.setTarget(100);
  TEST_ASSERT_TRUE(j.clamped);
  TEST_ASSERT_EQUAL_FLOAT(20, j.target);
}

void test_motion_stop_never_overshoots_far() {
  JointMotion j;
  j.vmax = 60; j.amax = 180;
  j.reset(0);
  j.setTarget(80);
  for (int i = 0; i < 60; i++) j.step(0.01f);    // now cruising at 60 deg/s
  float at = j.pos;
  j.stop();
  for (int i = 0; i < 200; i++) j.step(0.01f);
  TEST_ASSERT_TRUE(j.pos - at < 60.0f * 60.0f / (2 * 180.0f) + 1.5f);   // braking distance v^2 / 2a = 10 deg
  TEST_ASSERT_FALSE(j.moving());
}

void test_servo_cal() {
  ServoCal c;
  TEST_ASSERT_EQUAL(1500, c.pulse(0));
  TEST_ASSERT_EQUAL(2500, c.pulse(200));   // clamped to the calibrated maximum
  c.usPerDeg = -10;
  TEST_ASSERT_EQUAL(1400, c.pulse(10));    // negative scale = reversed direction
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_vectors_decode);
  RUN_TEST(test_vectors_encode);
  RUN_TEST(test_bad_crc_rejected);
  RUN_TEST(test_state_size);
  RUN_TEST(test_motion_reaches_target_within_limits);
  RUN_TEST(test_motion_clamps_to_limits);
  RUN_TEST(test_motion_stop_never_overshoots_far);
  RUN_TEST(test_servo_cal);
  return UNITY_END();
}
