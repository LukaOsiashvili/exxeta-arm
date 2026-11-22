// exxeta-arm web app: main script (plain ES modules, no build step, no framework).
//
// Role: one WebSocket to the `web_server` node. The server pushes a "state"
// message 20 times a second (newest message of every dataflow topic); the app
// renders it and sends commands back as {t:"cmd", d:{cmd: ...}}.
// Units on the wire are SI (radians, metres); degrees and millimetres appear
// only here, in the UI (PROJECT_SPEC 8.1).
//
// Only the client holding the control lock may send commands. STOP is the
// exception: it always goes through.
import { Viewer } from './viewer.js';

const $ = id => document.getElementById(id);
const deg = r => r * 180 / Math.PI, rad = d => d * Math.PI / 180;
const JOINTS = ['J1 BASE', 'J2 SHOULDER', 'J3 ELBOW', 'J4 WRIST', 'J5 ROLL', 'J6 GRIPPER'];
const MODE_NAMES = ['DISABLED', 'IDLE', 'MOVING', 'STOPPED', 'FAULT'];

let ws, me = null, controller = null, S = {}, limits = null;
let waypoints = [], logLines = [], lastMsgs = {}, targetOn = false, targetOk = false, ikSeq = 0, calUs = 1500, calZero = null, calRef = null;
const viewer = new Viewer($('canvas-wrap'));
viewer.load().catch(e => log('3D model failed to load: ' + e));

// ---------------------------------------------------------------- transport
function connect() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws${location.search}`);
  ws.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.t === 'hello') { me = m.id; $('phone-url').textContent = m.url; }
    else if (m.t === 'state') { S = m.d; controller = m.controller; me = m.you; render(m); }
    else if (m.t === 'ik') onIk(m.d);
  };
  ws.onclose = () => { S = {}; render({ clients: 0 }); setTimeout(connect, 1000); };
}
const haveControl = () => controller !== null && controller === me;
function send(t, d) { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t, d })); }
function cmd(d) { send('cmd', d); }
function log(text) {
  logLines.push(new Date().toLocaleTimeString() + '  ' + text);
  logLines = logLines.slice(-80);
  $('log').textContent = logLines.join('\n');
  $('log').scrollTop = 1e6;
}

// ------------------------------------------------------------------ render
function render(m) {
  const dev = S.device_status || {}, saf = S.safety_status || {}, bat = S.battery || {}, est = S.estimated_state, pl = S.planner_status || {};
  const online = !!S.device_status, connected = !!dev.connected;
  // header chips
  $('chip-source').textContent = !online ? 'NO SERVER' : dev.simulated ? 'SIMULATION' : (connected ? `HARDWARE · ${dev.transport} · ${dev.latency_ms} ms` : 'HARDWARE');
  $('chip-source').className = 'chip' + (dev.simulated ? ' invert' : '');
  const mode = !online ? 'OFFLINE' : !connected ? 'NOT CONNECTED' : (saf.mode_name || MODE_NAMES[dev.mode] || '?');
  $('chip-mode').textContent = mode;
  $('chip-mode').className = 'chip' + (['STOPPED', 'FAULT', 'OFFLINE'].includes(mode) ? ' alert' : '');
  const faults = dev.fault_names || [];
  $('chip-battery').textContent = connected ? `${bat.percent ?? '—'} % · ${(bat.voltage ?? 0).toFixed(1)} V` : '— %';
  $('chip-battery').className = 'chip mono' + (faults.includes('BATTERY_CRITICAL') ? ' alert' : faults.includes('BATTERY_WARN') ? ' warn' : '');
  $('btn-control').textContent = haveControl() ? 'RELEASE CONTROL' : (controller === null ? 'TAKE CONTROL' : 'TAKE CONTROL (IN USE)');
  $('btn-control').classList.toggle('active', haveControl());

  // banner: the single most important problem
  let banner = '', warn = false;
  if (!online) banner = 'NO CONNECTION TO THE PC SOFTWARE';
  else if (faults.includes('BATTERY_CRITICAL')) banner = 'BATTERY EMPTY: ARM IS PARKING. CHARGE NOW.';
  else if (faults.includes('LINK_LOST')) banner = 'LINK TO THE ARM WAS LOST. ARM IS HOLDING. ACKNOWLEDGE TO CONTINUE.';
  else if (saf.stop_latched || faults.includes('STOP_LATCHED')) banner = 'STOPPED. ACKNOWLEDGE TO CONTINUE.';
  else if (faults.includes('SERVO_POWER_OFF')) banner = 'SERVO POWER IS OFF: SWITCH ON THE MAIN SWITCH.';
  else if (dev.error) banner = dev.error;
  else if (faults.includes('BATTERY_WARN')) { banner = 'BATTERY LOW: FINISH AND RECHARGE.'; warn = true; }
  else if (bat.charging) { banner = 'CHARGER CONNECTED. UNPLUG WHEN FULL.'; warn = true; }
  $('banner').textContent = banner;
  $('banner').className = 'banner' + (banner ? '' : ' hidden') + (warn ? ' warn' : '');
  $('btn-clear').classList.toggle('hidden', !(saf.stop_latched || faults.includes('STOP_LATCHED') || faults.includes('LINK_LOST')));

  // messages from the nodes go to the log once
  for (const [key, text] of [['planner', pl.message], ['safety', saf.reason], ['recorder', (S.recorder_status || {}).message], ['device', dev.error]]) {
    if (text && lastMsgs[key] !== text) log(`${key}: ${text}`);
    lastMsgs[key] = text;
  }

  const ctl = haveControl();
  for (const id of ['btn-enable', 'btn-disable', 'btn-park']) $(id).disabled = !ctl || !connected;
  $('btn-enable').classList.toggle('active', connected && saf.mode > 0);

  if (saf.limits && !limits) { limits = saf.limits; buildJoints(); }
  if (est) {
    const measured = est.source.some(s => s === 'measured');
    viewer.setPose(est.q_cmd, measured ? est.q : null);       // ghost = measured pose when pots are fitted
    updateJoints(est);
  }
  const p = S.ee_pose;
  if (p) $('pose-readout').textContent =
    `X ${(p.x * 1000).toFixed(0).padStart(5)} mm\nY ${(p.y * 1000).toFixed(0).padStart(5)} mm\nZ ${(p.z * 1000).toFixed(0).padStart(5)} mm\nPITCH ${deg(p.pitch).toFixed(0).padStart(4)}°\nROLL ${deg(p.roll).toFixed(0).padStart(4)}°`;
  renderPoses(pl);
  renderPrograms(S.recorder_status || {});
  renderSetup(dev, pl);
}

// ------------------------------------------------------------------ joints
const dragging = new Set();
function buildJoints() {
  const box = $('joint-list');
  box.innerHTML = '';
  JOINTS.forEach((name, i) => {
    const lo = deg(limits.min[i]), hi = deg(limits.max[i]);
    const el = document.createElement('div');
    el.className = 'joint';
    el.innerHTML = `<div><span class="name">${name}</span><span class="tag" id="jt${i}"></span></div>
      <input type="number" id="jn${i}" min="${lo.toFixed(0)}" max="${hi.toFixed(0)}" step="1">
      <div class="slider-wrap"><span>${lo.toFixed(0)}°</span><input type="range" id="js${i}" min="${lo}" max="${hi}" step="0.5"><span>${hi.toFixed(0)}°</span></div>`;
    box.appendChild(el);
    const slider = $(`js${i}`), numInput = $(`jn${i}`);
    let lastSend = 0;
    const go = v => { const q = currentQ(); q[i] = rad(Math.min(hi, Math.max(lo, v))); cmd({ cmd: 'move_joints', q, speed: 1.0 }); };
    slider.addEventListener('pointerdown', () => dragging.add(i));
    slider.addEventListener('input', () => {
      numInput.value = (+slider.value).toFixed(0);
      if (performance.now() - lastSend > 150) { lastSend = performance.now(); go(+slider.value); }
    });
    slider.addEventListener('change', () => { go(+slider.value); setTimeout(() => dragging.delete(i), 600); });
    numInput.addEventListener('focus', () => dragging.add(i));
    numInput.addEventListener('blur', () => dragging.delete(i));
    numInput.addEventListener('change', () => { go(+numInput.value); numInput.blur(); });
  });
  $('limits').textContent = JOINTS.map((n, i) => `${n}: ${deg(limits.min[i]).toFixed(0)}° … ${deg(limits.max[i]).toFixed(0)}°`).join('\n');
  $('limits').style.whiteSpace = 'pre';
  $('cal-joint').innerHTML = JOINTS.map((n, i) => `<option value="${i}">${n}</option>`).join('');
}
function currentQ() { return (S.safety_status && S.safety_status.q_safe ? S.safety_status.q_safe : S.estimated_state.q_cmd).slice(); }
function updateJoints(est) {
  if (!limits) return;
  for (let i = 0; i < 6; i++) {
    $(`jt${i}`).textContent = est.source[i] === 'measured' ? `MEASURED ${deg(est.q[i]).toFixed(1)}°` : 'ESTIMATED';
    if (dragging.has(i)) continue;
    $(`js${i}`).value = deg(est.q_cmd[i]);
    if (document.activeElement !== $(`jn${i}`)) $(`jn${i}`).value = deg(est.q_cmd[i]).toFixed(0);
  }
}

// ------------------------------------------------------- hold-to-move jogging
let jogTimer = null;
function startJog(vel) { stopJog(); const tick = () => cmd({ cmd: 'jog_cart', vel }); tick(); jogTimer = setInterval(tick, 100); }
function stopJog() { if (jogTimer) { clearInterval(jogTimer); jogTimer = null; } }
const JOGS = [['X−', [-1, 0, 0, 0, 0, 0]], ['X+', [1, 0, 0, 0, 0, 0]], ['Y−', [0, -1, 0, 0, 0, 0]], ['Y+', [0, 1, 0, 0, 0, 0]],
  ['Z−', [0, 0, -1, 0, 0, 0]], ['Z+', [0, 0, 1, 0, 0, 0]], ['PITCH−', [0, 0, 0, -1, 0, 0]], ['PITCH+', [0, 0, 0, 1, 0, 0]],
  ['ROLL−', [0, 0, 0, 0, -1, 0]], ['ROLL+', [0, 0, 0, 0, 1, 0]], ['CLOSE', [0, 0, 0, 0, 0, -1]], ['OPEN', [0, 0, 0, 0, 0, 1]]];
for (const [label, vel] of JOGS) {
  const b = document.createElement('button');
  b.textContent = label;
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); b.classList.add('held'); startJog(vel); });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(ev, () => { b.classList.remove('held'); stopJog(); });
  $('jog-grid').appendChild(b);
}
// on-screen joysticks (phones): drag inside the square; the distance from the centre sets the speed
function stick(el, toVel) {
  const knob = el.querySelector('.knob');
  const move = e => {
    const r = el.getBoundingClientRect();
    let x = ((e.clientX - r.left) / r.width - 0.5) * 2, y = -((e.clientY - r.top) / r.height - 0.5) * 2;
    x = Math.max(-1, Math.min(1, x)); y = Math.max(-1, Math.min(1, y));
    knob.style.left = (50 + x * 38) + '%'; knob.style.top = (50 - y * 38) + '%';
    const dz = v => Math.abs(v) < 0.15 ? 0 : v;                 // small dead zone in the middle
    startJog(toVel(dz(x), dz(y)));
  };
  el.addEventListener('pointerdown', e => { e.preventDefault(); el.setPointerCapture(e.pointerId); move(e); el.onpointermove = move; });
  const end = () => { el.onpointermove = null; knob.style.left = knob.style.top = '50%'; stopJog(); };
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) el.addEventListener(ev, end);
}
stick($('stick-xy'), (x, y) => [y, -x, 0, 0, 0, 0]);          // up = forward (+X), right = -Y
stick($('stick-zp'), (x, y) => [0, 0, y, x, 0, 0]);           // up = +Z, right = pitch down

// ------------------------------------------------------------------- poses
let posesKey = '';
function renderPoses(pl) {
  const poses = pl.poses || {}, key = Object.keys(poses).join('|');
  if (key === posesKey) return;
  posesKey = key;
  $('pose-list').innerHTML = '';
  for (const name of Object.keys(poses)) {
    const builtIn = (pl.built_in || []).includes(name);
    const el = document.createElement('div');
    el.className = 'item';
    el.innerHTML = `<div class="label">${name}${builtIn ? '<small>built-in</small>' : ''}</div><button data-a="go">GO</button>` +
      (builtIn ? '' : '<button data-a="ren">RENAME</button><button data-a="del">DELETE</button>');
    el.querySelector('[data-a=go]').onclick = () => cmd({ cmd: 'move_pose', name });
    if (!builtIn) {
      el.querySelector('[data-a=del]').onclick = () => cmd({ cmd: 'pose_delete', name });
      el.querySelector('[data-a=ren]').onclick = () => { const to = prompt('New name', name); if (to) cmd({ cmd: 'pose_rename', name, to: to.trim() }); };
    }
    $('pose-list').appendChild(el);
  }
}
$('btn-pose-save').onclick = () => { const name = $('pose-name').value.trim(); if (name) { cmd({ cmd: 'pose_save', name }); $('pose-name').value = ''; } };

// ---------------------------------------------------------------- programs
function renderWaypoints() {
  $('wp-list').innerHTML = '';
  waypoints.forEach((w, i) => {
    const el = document.createElement('div');
    el.className = 'item';
    el.innerHTML = `<div class="label">${i + 1}<small>${w.q.slice(0, 5).map(a => deg(a).toFixed(0)).join(' · ')}° grip ${deg(w.gripper).toFixed(0)}°</small></div>
      <input type="number" title="speed 0.1-1" min="0.1" max="1" step="0.1" value="${w.speed}">
      <input type="number" title="dwell s" min="0" max="30" step="0.5" value="${w.dwell}">
      <button data-a="up">↑</button><button data-a="set">SET</button><button data-a="del">×</button>`;
    const [sp, dw] = el.querySelectorAll('input');
    sp.onchange = () => { w.speed = +sp.value; };
    dw.onchange = () => { w.dwell = +dw.value; };
    el.querySelector('[data-a=up]').onclick = () => { if (i > 0) { [waypoints[i - 1], waypoints[i]] = [waypoints[i], waypoints[i - 1]]; renderWaypoints(); } };
    el.querySelector('[data-a=set]').onclick = () => { const q = currentQ(); w.q = q; w.gripper = q[5]; renderWaypoints(); };   // overwrite with the current pose
    el.querySelector('[data-a=del]').onclick = () => { waypoints.splice(i, 1); renderWaypoints(); };
    $('wp-list').appendChild(el);
  });
}
$('btn-wp-add').onclick = () => { if (!S.estimated_state) return; const q = currentQ(); waypoints.push({ type: 'joint', q, gripper: q[5], speed: 0.5, dwell: 0 }); renderWaypoints(); };
$('btn-prog-save').onclick = () => { const name = $('prog-name').value.trim(); if (name && waypoints.length) cmd({ cmd: 'program_save', name, steps: waypoints }); };
let progKey = '', loadedProgram = '';
function renderPrograms(rs) {
  const key = (rs.programs || []).join('|') + '#' + (rs.recordings || []).join('|');
  if (key !== progKey) {
    progKey = key;
    $('prog-list').innerHTML = '';
    for (const [name, kind] of [...(rs.programs || []).map(n => [n, 'program']), ...(rs.recordings || []).map(n => [n, 'recording'])]) {
      const el = document.createElement('div');
      el.className = 'item';
      el.innerHTML = `<div class="label">${name}<small>${kind}</small></div><button data-a="play">PLAY</button>` +
        (kind === 'program' ? '<button data-a="edit">EDIT</button>' : '') + '<button data-a="del">×</button>';
      el.querySelector('[data-a=play]').onclick = () => cmd({ cmd: 'play', name, speed: +$('play-speed').value, loop: $('play-loop').checked });
      el.querySelector('[data-a=del]').onclick = () => { if (confirm(`Delete ${name}?`)) cmd({ cmd: 'program_delete', name }); };
      if (kind === 'program') el.querySelector('[data-a=edit]').onclick = () => { loadedProgram = ''; $('prog-name').value = name; cmd({ cmd: 'program_load', name }); };
      $('prog-list').appendChild(el);
    }
  }
  if (rs.program && rs.program.name === $('prog-name').value && loadedProgram !== rs.program.name) {   // EDIT pressed: load into the editor
    loadedProgram = rs.program.name;
    waypoints = rs.program.steps.map(s => ({ type: 'joint', speed: 0.5, dwell: 0, gripper: s.q ? s.q[5] : 0, ...s }));
    renderWaypoints();
  }
  $('play-status').textContent = rs.playing ? `PLAYING ${rs.playing}: ${rs.index + (rs.total ? 1 : 0)} / ${rs.total}${rs.paused ? ' (PAUSED)' : ''}` : '';
  $('btn-rec').textContent = rs.recording ? '■ STOP RECORDING' : '● RECORD';
  $('btn-rec').classList.toggle('active', !!rs.recording);
  $('rec-status').textContent = rs.recording ? `${rs.recording_name}: ${rs.samples} samples` : '';
}
$('btn-pause').onclick = () => cmd({ cmd: 'pause' });
$('btn-resume').onclick = () => cmd({ cmd: 'resume' });
$('btn-step').onclick = () => cmd({ cmd: 'step' });
$('btn-play-stop').onclick = () => cmd({ cmd: 'play_stop' });
$('btn-rec').onclick = () => cmd({ cmd: (S.recorder_status || {}).recording ? 'rec_stop' : 'rec_start' });

// ------------------------------------------------------------------- setup
let portsKey = null;
function renderSetup(dev, pl) {
  const ports = dev.ports || [], key = ports.join('|');
  if (key !== portsKey) { portsKey = key; $('conn-port').innerHTML = ports.length ? ports.map(p => `<option>${p}</option>`).join('') : '<option value="">NO SERIAL PORT FOUND</option>'; }
  $('conn-info').textContent = dev.simulated ? 'Simulation is running: there is no arm to connect to.' :
    dev.connected ? `Connected to "${dev.name}", firmware ${dev.firmware}, ${dev.transport}, link ${dev.link_quality} %, ${dev.latency_ms} ms` : (dev.error || 'Not connected.');
  if (pl.speed_scale !== undefined && document.activeElement !== $('speed')) { $('speed').value = Math.round(pl.speed_scale * 100); $('speed-out').textContent = $('speed').value + ' %'; }
}
$('conn-transport').onchange = () => { const wifi = $('conn-transport').value === 'wifi'; $('conn-port').classList.toggle('hidden', wifi); $('conn-host').classList.toggle('hidden', !wifi); };
$('btn-connect').onclick = () => cmd({ cmd: 'connect', transport: $('conn-transport').value, port: $('conn-port').value, host: $('conn-host').value.trim() });
$('btn-disconnect').onclick = () => cmd({ cmd: 'disconnect' });
$('speed').oninput = () => { $('speed-out').textContent = $('speed').value + ' %'; };
$('speed').onchange = () => cmd({ cmd: 'set_speed', scale: $('speed').value / 100 });
$('btn-wifi').onclick = () => { cmd({ cmd: 'set_wifi', ssid: $('wifi-ssid').value, password: $('wifi-pass').value }); cmd({ cmd: 'save_config' }); log('Wi-Fi settings sent. Restart the arm to join the network.'); };

// calibration wizard (PROJECT_SPEC 7.4): two points fix the pulse-to-angle line of one servo
function calJog(d) { calUs = Math.max(400, Math.min(2600, calUs + d)); $('cal-us').textContent = calUs + ' µs'; cmd({ cmd: 'jog_pulse', joint: +$('cal-joint').value, us: calUs }); }
document.querySelectorAll('[data-cal]').forEach(b => { b.onclick = () => calJog(+b.dataset.cal); });
$('cal-joint').onchange = () => { calZero = calRef = null; calUs = 1500; $('cal-us').textContent = '1500 µs'; $('cal-info').textContent = 'Jog until the notch lines up with the mark at 0°, then press SET 0°.'; };
$('btn-cal-zero').onclick = () => { calZero = calUs; $('cal-info').textContent = `0° = ${calZero} µs. Now jog to the reference notch and press SET REF.`; };
$('btn-cal-ref').onclick = () => { calRef = { us: calUs, deg: +$('cal-ref').value }; $('cal-info').textContent = `${calRef.deg}° = ${calRef.us} µs. Press SAVE TO ARM.`; };
$('btn-cal-save').onclick = () => {
  if (calZero === null || !calRef || !calRef.deg || calRef.us === calZero) { $('cal-info').textContent = 'Set both points first (they must differ).'; return; }
  const usPerDeg = (calRef.us - calZero) / calRef.deg;
  cmd({ cmd: 'set_servo_cal', joint: +$('cal-joint').value, min_us: 500, max_us: 2500, zero_us: calZero, us_per_deg: usPerDeg });
  cmd({ cmd: 'save_config' });
  $('cal-info').textContent = `Saved: 0° = ${calZero} µs, ${usPerDeg.toFixed(2)} µs per degree (negative = reversed direction).`;
};

// --------------------------------------------------------------- IK target
$('btn-target').onclick = () => {
  targetOn = !targetOn;
  $('btn-target').classList.toggle('active', targetOn);
  $('target-bar').classList.toggle('hidden', !targetOn);
  viewer.showTarget(targetOn, S.ee_pose);
  if (targetOn) queryIk();
};
function targetPose() { const t = viewer.target.position, p = S.ee_pose || { pitch: 2.2, roll: 0 }; return [t.x, t.y, t.z, p.pitch, p.roll]; }
function queryIk() { send('ik', { id: ++ikSeq, pose: targetPose(), relax: true }); }
let ikThrottle = 0;
viewer.onTargetMove = () => { if (performance.now() - ikThrottle > 80) { ikThrottle = performance.now(); queryIk(); } };
function onIk(r) {
  if (r.id !== ikSeq) return;
  targetOk = r.ok;
  $('target-bar').classList.toggle('bad', !r.ok);
  $('target-info').textContent = r.ok ? (r.pitch_relaxed ? 'REACHABLE (HAND ANGLE WILL CHANGE)' : 'REACHABLE') : 'UNREACHABLE: ' + r.reason.toUpperCase();
  $('btn-target-go').disabled = !r.ok;
  viewer.pendingQ = r.ok ? r.q : null;
}
$('btn-target-go').onclick = () => { if (targetOk && viewer.pendingQ) cmd({ cmd: 'move_joints', q: viewer.pendingQ }); };

// ------------------------------------------------------------- top-level UI
$('btn-control').onclick = () => send(haveControl() ? 'release' : 'take_control');
$('btn-enable').onclick = () => cmd({ cmd: 'enable' });
$('btn-disable').onclick = () => cmd({ cmd: 'disable' });
$('btn-park').onclick = () => cmd({ cmd: 'park' });
$('btn-stop').onclick = () => { stopJog(); cmd({ cmd: 'stop' }); };
$('btn-clear').onclick = () => cmd({ cmd: 'clear' });
document.querySelectorAll('.tabs button').forEach(b => {
  b.onclick = () => {
    document.querySelectorAll('.tabs button').forEach(x => x.classList.toggle('active', x === b));
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('hidden', t.id !== 'tab-' + b.dataset.tab));
  };
});
document.querySelectorAll('[data-view]').forEach(b => { b.onclick = () => viewer.setView(b.dataset.view); });
// keyboard: Space or Esc = STOP, from anywhere (PROJECT_SPEC 8.3)
window.addEventListener('keydown', e => {
  if (e.code === 'Escape' || (e.code === 'Space' && !['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName))) {
    e.preventDefault(); stopJog(); cmd({ cmd: 'stop' });
  }
});
window.addEventListener('blur', stopJog);
connect();
