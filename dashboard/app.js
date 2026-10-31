'use strict';
/* PicoCar IS23 control room. Polls the local server (dashboard.py) for a
   snapshot and renders it; in demo mode the same snapshot is simulated
   here. Every value from the robot is escaped before it reaches the page. */

const $ = id => document.getElementById(id);
const FRESH_S = 3;
const WINDOW_S = 60;
/* Robot geometry for the bypass path: copy from subsystem2/motion_cfg.h
   once measured (phase D of docs/SETUP_AND_TEST.md). */
const TRACK_MM = 106;                      /* MOTION_TRACK_MM */
const MM_PER_COUNT = 209.309 / 2536;       /* MOTION_WHEEL_CIRC_UM / 1000 / MOTION_COUNTS_PER_REV */
const BYPASS_STATES = ['OBSTACLE_HANDLING', 'AVOIDING', 'LINE_RECOVERY'];
const MOVES = ['Left', 'Right', 'Straight', 'U-turn'];
const REJECT = {2: 'wrong element count', 3: 'bad wide/narrow ratio', 4: 'unknown pattern', 5: 'no start/stop *', 6: 'wrong length', 7: 'too long'};
const MOTION = ['stationary', 'accelerating', 'turning', 'climbing', 'descending', 'impact', 'cruising', 'braking'];
const FAULTS = [[1, 'Impact', 'Acceleration exceeded the impact threshold.'], [2, 'Obstacle too close', 'Obstacle closer than 8 cm.'],
  [4, 'Motor stall', 'High motor effort with no wheel movement.'], [8, 'Excessive tilt', 'Pitch or roll beyond 30°.'],
  [16, 'Emergency stop', 'An E-stop was requested.'], [32, 'Heartbeat timeout', 'A monitored task stopped responding.'],
  [64, 'Encoder warning', 'Encoder counts jumped. Report-only.'], [128, 'IMU warning', 'IMU reads failed. Report-only after start-up.'],
  [256, 'No echo', 'No ultrasonic echo. Report-only.'], [512, 'Wi-Fi loss', 'Wi-Fi loss reported. Report-only.'],
  [1024, 'Start-up self-test failed', 'A start-up check did not pass.']];
const LOGS = {31: 'Barcode rejected', 32: 'IR calibration (left)', 33: 'IR calibration (right)', 34: 'IR calibration (barcode)',
  41: 'Heartbeat missed', 42: 'Last reset was the watchdog', 51: 'Motion class', 52: 'Ramp, not a hump', 53: 'IMU start result',
  54: 'I²C scan', 61: 'Start refused: not calibrated', 62: 'Calibration / self-test result', 63: 'Barcode latched',
  64: 'Barcode not in the map', 65: 'Map stored', 66: 'Map refused', 67: 'Run summary', 68: 'Parameter set',
  69: 'Mission queue full', 70: 'Motion queue full', 90: 'IMU bus start', 91: 'IMU identity read', 92: 'IMU identity value',
  93: 'IMU configuration', 100: 'IMU accel X', 101: 'IMU accel Y', 102: 'IMU accel Z', 103: 'IMU pitch', 104: 'IMU roll',
  110: 'IMU communication error', 111: 'I²C line levels', 112: 'IMU read recovered'};
const STATE_TITLES = {BOOT: 'Running start-up checks', IDLE: 'Standing by', CALIBRATING: 'Calibrating', LINE_FOLLOW: 'Following the line',
  EXECUTE_NAV: 'Turning at a junction', OBSTACLE_HANDLING: 'Scanning an obstacle', AVOIDING: 'Driving around the obstacle',
  LINE_RECOVERY: 'Finding the line again', LINE_SEARCH: 'Searching for the line', MISSION_COMPLETE: 'Run complete', FAULT: 'Safety stop active'};

let mode = 'offline', token = '', snap = {}, filter = 'all', pollBusy = false, demo = null, lastEventSig = '';

/* ---------- helpers ---------- */
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
const isNum = x => typeof x === 'number' && Number.isFinite(x);
const fmt = (x, d = 0) => isNum(x) ? x.toLocaleString(undefined, {maximumFractionDigits: d, minimumFractionDigits: d}) : '—';
const clock = t => new Date(t * 1000).toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'});
const nowS = () => mode === 'local' && isNum(snap.now) ? snap.now + (Date.now() / 1000 - snap.fetched) : Date.now() / 1000;
function toast(text) { $('toast').textContent = text; $('toast').style.display = 'block'; clearTimeout(toast.t); toast.t = setTimeout(() => $('toast').style.display = 'none', 4500); }
function band(value, limits) { return value <= limits[0] ? 'a' : value <= limits[1] ? 'b' : value <= limits[2] ? 'c' : 'f'; }
const bandText = b => b === 'f' ? 'outside C' : 'band ' + b.toUpperCase();
const ev = e => e.payload || {};
const isType = (e, t) => e.topic.endsWith('/event') && ev(e).type === t;
const isLog = (e, c) => e.topic.endsWith('/event') && ev(e).type === 'LOG' && ev(e).code === c;
const charOf = d => typeof d === 'string' ? d : isNum(d) && d >= 32 && d < 127 ? String.fromCharCode(d) : '?';

/* ---------- event descriptions ---------- */
function describe(e) {
  const p = ev(e), type = p.type || '', kind = e.topic.split('/').pop();
  if (kind === 'ack') {
    const c = (snap.commands || []).find(x => x.id === p.id);
    return {cat: 'commands', title: (c ? c.label + ' ' : 'Command ') + (p.ok === true ? 'accepted' : 'rejected'),
      detail: `#${p.id ?? '—'}${p.err ? ' · ' + p.err : ''}`, sym: p.ok ? '✓' : '!', tone: p.ok ? '' : 'fault'};
  }
  if (type === 'FAULT') {
    const m = FAULTS.filter(([b]) => (Number(p.data) & b) !== 0);
    return {cat: 'faults', title: m.map(x => x[1]).join(' + ') || `Fault ${p.data}`, detail: `Code ${p.data} · ${m.map(x => x[2]).join(' ') || 'Unknown fault.'}`,
      sym: '!', tone: m.length && m.every(x => [64, 128, 256, 512].includes(x[0])) ? 'warn' : 'fault'};
  }
  if (type === 'STATE') return {cat: 'state', title: 'State → ' + p.data, detail: STATE_TITLES[p.data] || '', sym: '↗', tone: p.data === 'FAULT' ? 'fault' : ''};
  if (type === 'LOG') {
    let detail = `Value ${p.data}`;
    if (p.code === 63) detail = `${charOf(p.data & 0xFFFF)} → ${MOVES[p.data >> 16] || '?'} at the next junction`;
    if (p.code === 64) detail = `'${charOf(p.data)}' is not in the map: ignored`;
    if (p.code === 31) detail = REJECT[p.data] || detail;
    if (p.code === 51) detail = MOTION[p.data] || detail;
    if (p.code === 67) detail = `Highest hump ${p.data} mm`;
    if (p.code === 53 || p.code === 62) detail = `0x${(p.data >>> 0).toString(16).padStart(8, '0')}`;
    const cat = [63, 64, 31, 65, 66, 67].includes(p.code) ? 'mission' : 'diagnostic';
    return {cat, title: LOGS[p.code] || `Log ${p.code}`, detail, sym: '·', tone: [31, 41, 61, 66, 69, 70, 110].includes(p.code) ? 'warn' : ''};
  }
  const mission = {BARCODE: [`Barcode '${charOf(p.data)}' read`, ''], JUNCTION: ['Cross-junction', ''], LINE_LOST: ['Line lost', ''], LINE_FOUND: ['Line found', ''],
    OBSTACLE: ['Obstacle ahead', `${fmt(p.data / 10, 1)} cm`], SLOW_DOWN: ['Slowing for an obstacle', `${fmt(p.data / 10, 1)} cm`],
    PROFILE_DONE: ['Obstacle profiled', `width ${p.data} mm`], CONTINUE: ['Path clear, continuing', ''], BYPASS_DONE: ['Bypass manoeuvre done', ''],
    RECOVERY_DONE: ['Back on the line', ''], RECOVERY_FAILED: ['Could not find the line after the bypass', ''], BLOCKED_TIMEOUT: ['Path blocked for 10 s', ''],
    HUMP_START: ['Hump: climbing', ''], HUMP_END: ['Hump finished', `peak ${p.data} mm`], CAL_DONE: ['Calibration step done', p.data ? 'ok' : 'failed'],
    MOTION_DONE: ['Move done', ''], MOTION_FAILED: ['Move did not complete', ''], REMOTE_MAP: ['Map received', ''], SAFETY_OK: ['Safety latch cleared', '']};
  if (mission[type]) {
    const bad = ['RECOVERY_FAILED', 'BLOCKED_TIMEOUT', 'MOTION_FAILED'].includes(type) || (type === 'CAL_DONE' && !p.data);
    return {cat: type.startsWith('REMOTE') || type === 'SAFETY_OK' ? 'commands' : 'mission', title: mission[type][0], detail: mission[type][1], sym: bad ? '!' : '↗', tone: bad ? 'warn' : ''};
  }
  const names = {BTN_START_STOP: 'Start/stop button', BTN_CALIBRATE: 'Calibrate button', BTN_RESET: 'Reset button (held)', REMOTE_START: 'Remote start',
    REMOTE_STOP: 'Remote stop', REMOTE_RESET: 'Remote reset', REMOTE_SET: 'Parameter or bench move', MOTION_CLASS: 'Motion class'};
  return {cat: type.startsWith('REMOTE') || type.startsWith('BTN') ? 'commands' : 'mission', title: names[type] || type.replaceAll('_', ' ') || 'Event',
    detail: p.data !== undefined ? 'Value ' + p.data : '', sym: '↗', tone: ''};
}

/* ---------- charts ---------- */
function chart(id, series, colors, refs = [], floor = null) {
  const el = $(id), now = nowS(), h = (snap.history || []).filter(x => now - x.t <= WINDOW_S);
  const W = 300, H = 80, X = t => W - (now - t) / WINDOW_S * W;
  const vals = h.flatMap(x => series.map(s => x[s])).filter(isNum).concat(refs);
  let lo = Math.min(floor ?? Infinity, ...vals), hi = Math.max(...vals);
  if (!vals.length) { lo = 0; hi = 1; }
  if (hi - lo < 10) { hi = lo + 10; }
  const Y = v => H - 6 - (v - lo) / (hi - lo) * (H - 12);
  let svg = refs.map(r => `<line x1="0" x2="${W}" y1="${Y(r)}" y2="${Y(r)}" stroke="#ead9b8" stroke-dasharray="4 4"/>`).join('');
  if (lo < 0 && hi > 0) svg += `<line x1="0" x2="${W}" y1="${Y(0)}" y2="${Y(0)}" stroke="#e6eae3"/>`;
  series.forEach((s, i) => {
    let d = '', pen = false;
    h.forEach(x => { if (isNum(x[s])) { d += `${pen ? 'L' : 'M'}${X(x.t).toFixed(1)},${Y(x[s]).toFixed(1)}`; pen = true; } else pen = false; });
    if (d) svg += `<path d="${d}" stroke="${colors[i]}" stroke-width="1.8" fill="none" stroke-linejoin="round" vector-effect="non-scaling-stroke"/>`;
  });
  el.innerHTML = svg || '';
  return [lo, hi];
}

/* ---------- derived results ---------- */
function bypassResult() {
  const evs = (snap.events || []).filter(e => e.topic.endsWith('/event'));
  let start = null;
  for (let i = evs.length - 1; i >= 0; i--) if (isType(evs[i], 'OBSTACLE')) { start = i; break; }
  if (start === null) return null;
  const s = ev(evs[start]);
  for (let j = start + 1; j < evs.length; j++) {
    const p = ev(evs[j]);
    if (['RECOVERY_DONE', 'RECOVERY_FAILED', 'BLOCKED_TIMEOUT', 'CONTINUE'].includes(p.type) || (p.type === 'STATE' && p.data === 'FAULT')) {
      const sec = isNum(p.t_ms) && isNum(s.t_ms) ? (p.t_ms - s.t_ms) / 1000 : (evs[j].time - evs[start].time);
      return {sec, end: p.type === 'STATE' ? 'FAULT' : p.type, at: evs[j].time};
    }
  }
  return {sec: null, end: 'in progress', at: evs[start].time};
}

function scans() {
  const rows = [];
  for (const e of snap.events || []) {
    if (isType(e, 'BARCODE')) rows.push({t: e.time, c: charOf(ev(e).data), res: 'decoded', tone: ''});
    else if (isLog(e, 63) || isLog(e, 64)) {
      const c = isLog(e, 63) ? charOf(ev(e).data & 0xFFFF) : charOf(ev(e).data);
      const r = [...rows].reverse().find(x => x.c === c && x.res === 'decoded');
      const res = isLog(e, 63) ? `latched → ${MOVES[ev(e).data >> 16] || '?'} at next junction` : 'not in the map: ignored';
      if (r) { r.res = res; r.tone = isLog(e, 63) ? 'ok' : 'warn'; } else rows.push({t: e.time, c, res, tone: isLog(e, 63) ? 'ok' : 'warn'});
    } else if (isLog(e, 31)) rows.push({t: e.time, c: '—', res: 'rejected: ' + (REJECT[ev(e).data] || ev(e).data), tone: 'bad'});
  }
  return rows;
}

function humps() {
  const rows = [], evs = snap.events || [];
  let start = null;
  for (const e of evs) {
    if (isType(e, 'HUMP_START')) start = e;
    else if (isType(e, 'HUMP_END')) {
      const p = ev(e), s = start ? ev(start) : null;
      const t0 = start ? start.time : e.time - 2;
      const pitch = (snap.history || []).filter(x => x.t >= t0 && x.t <= e.time && isNum(x.p)).map(x => x.p);
      rows.push({t: e.time, peak: p.data, pitch: pitch.length ? Math.max(...pitch) : null,
        dur: s && isNum(s.t_ms) && isNum(p.t_ms) ? (p.t_ms - s.t_ms) / 1000 : null});
      start = null;
    }
  }
  return rows;
}

function pathPoints(ep) {
  /* Dead reckoning from the cumulative encoder counts (exact wheel
     distances, whatever the 5 Hz sampling misses); wheel speeds are the
     fallback if a sample has no counts. */
  const pts = [[0, 0]];
  let x = 0, y = 0, th = 0;
  const s = ep?.samples || [];
  for (let i = 1; i < s.length; i++) {
    const a = s[i - 1], b = s[i];
    let dl, dr;
    if ([a.el, a.er, b.el, b.er].every(isNum)) { dl = (b.el - a.el) * MM_PER_COUNT; dr = (b.er - a.er) * MM_PER_COUNT; }
    else {
      const dt = isNum(a.tms) && isNum(b.tms) ? (b.tms - a.tms) / 1000 : 0.2;
      if (!(dt > 0 && dt < 1.5) || !isNum(b.vl) || !isNum(b.vr)) continue;
      dl = b.vl * dt; dr = b.vr * dt;
    }
    if (Math.abs(dl) > 2000 || Math.abs(dr) > 2000) continue;          /* counter reset or glitch */
    const ds = (dl + dr) / 2, dth = (dr - dl) / TRACK_MM;
    x += ds * Math.cos(th + dth / 2); y += ds * Math.sin(th + dth / 2); th += dth;
    pts.push([x, y]);
  }
  return pts;
}

/* ---------- rendering ---------- */
function render() {
  const t = snap.telemetry || {}, s = snap.status || {};
  const age = snap.received ? nowS() - snap.received : Infinity;
  const fresh = mode === 'demo' || (mode === 'local' && age <= FRESH_S && s.online !== false);
  const reading = Boolean(snap.telemetry), state = t.state || 'NO DATA';
  const controls = mode === 'local' && snap.broker, idle = state === 'IDLE';

  /* status */
  $('state').textContent = mode === 'local' && !fresh && reading ? 'STALE · ' + state : state;
  $('state').className = 'state ' + (state === 'FAULT' ? 'bad' : idle || state === 'LINE_FOLLOW' ? 'good' : '');
  $('stateTitle').textContent = reading ? (STATE_TITLES[state] || state.replaceAll('_', ' ')) : 'Waiting for connection';
  $('stateDetail').textContent = mode === 'local' && !fresh && reading ? 'Readings are stale. The current robot state is unknown.'
    : state === 'FAULT' ? 'Find the cause in the event history before resetting.'
    : idle ? (s.map ? 'Map set. Calibrate (GP21) if needed, then start.' : 'Send the barcode map, calibrate (GP21), then start.')
    : reading ? 'Readings update automatically as the robot reports.' : 'Live state will appear here once telemetry arrives.';
  $('modePill').textContent = mode === 'demo' ? 'DEMO' : fresh ? 'LIVE' : mode === 'local' ? 'WAITING' : 'OFFLINE';
  $('modePill').className = 'pill ' + (mode === 'demo' ? 'demo' : fresh ? 'live' : '');
  $('connectionDot').className = 'dot ' + (fresh ? 'on' : '');
  $('robotLed').setAttribute('fill', fresh ? '#b5d78d' : '#aab9ad');
  $('connectionText').textContent = mode === 'demo' ? 'Simulated readings · no robot control' : fresh ? 'Receiving live telemetry' : snap.error || 'Waiting for robot telemetry';
  $('lastSeen').textContent = mode === 'demo' ? 'simulated' : age === Infinity ? '—' : age < 1 ? 'just now' : Math.floor(age) + 's ago';
  $('rssi').textContent = isNum(s.rssi) ? fmt(s.rssi) + ' dBm' : '—';
  $('uptime').textContent = isNum(s.uptime_s) ? `${Math.floor(s.uptime_s / 60)}m ${s.uptime_s % 60}s` : '—';
  $('drops').textContent = fmt(s.drops);
  const vl = t.v_mm_s?.[0], vr = t.v_mm_s?.[1];
  $('activity').textContent = !reading ? '—' : !isNum(vl) || (Math.abs(vl) < 5 && Math.abs(vr) < 5) ? 'STOPPED'
    : Math.sign(vl) !== Math.sign(vr) ? 'SPINNING' : Math.abs(vl - vr) > 60 ? 'TURNING' : vl < 0 ? 'REVERSING' : 'DRIVING';
  $('wheelL').setAttribute('fill', isNum(vl) && Math.abs(vl) > 5 ? '#4f7a63' : '#293e35');
  $('wheelR').setAttribute('fill', isNum(vr) && Math.abs(vr) > 5 ? '#4f7a63' : '#293e35');

  /* controls */
  $('estop').disabled = !controls; $('stop').disabled = !controls;
  $('reset').disabled = !(controls && fresh);
  $('start').disabled = !(controls && fresh && idle);
  $('mapSend').disabled = !(controls && fresh && idle);
  document.querySelectorAll('[data-bench]').forEach(b => b.disabled = !(controls && fresh && idle));
  const cmds = snap.commands || [], c = cmds[cmds.length - 1];
  $('commandStatus').textContent = c ? `${c.label} #${c.id} · ${c.result}${c.ack?.err ? ' (' + c.ack.err + ')' : ''}${c.error ? ' · ' + c.error : ''}`
    : controls ? (idle ? 'Ready. Start, map and bench moves need IDLE and fresh telemetry.' : 'E-stop and stop are always available.')
    : mode === 'local' ? 'Waiting for MQTT messages; controls are unavailable.' : 'Controls unlock in the connected local dashboard.';
  $('commandList').innerHTML = cmds.length ? cmds.slice(-8).reverse().map(x => {
    const tone = x.result === 'accepted' ? 'ok' : x.result === 'pending' ? '' : 'bad';
    return `<div class="command-row"><div><b>${esc(x.label)}</b><small>#${esc(x.id)} · ${esc(clock(x.time))}</small></div><span class="tag ${tone}">${esc(x.result)}${x.ack?.err ? ': ' + esc(x.ack.err) : ''}</span></div>`;
  }).join('') : '<p class="muted">No commands sent.</p>';

  /* mission */
  const m = s.map;
  $('mapNow').innerHTML = m && typeof m === 'object' ? ['left', 'right', 'straight', 'uturn'].map((k, i) => `${esc(m[k])}=${MOVES[i]}`).join(' · ') : '<span class="tag warn">none: barcodes are ignored</span>';
  $('lastBarcode').textContent = t.barcode ?? '—';
  $('navNow').textContent = t.nav ? `'${t.nav}'` : 'nothing latched';

  /* graphs and numbers */
  $('speedL').textContent = fmt(vl); $('speedR').textContent = fmt(vr);
  const sp = chart('speedChart', ['vl', 'vr'], ['#699878', '#c8a565'], [], 0);
  $('speedAxis').textContent = `${fmt(sp[0])}…${fmt(sp[1])}`;
  $('obstacle').textContent = fmt(t.obstacle?.d_cm, 1);
  const rg = chart('rangeChart', ['d'], ['#5f8fa0'], [20, 40], 0);
  $('rangeAxis').textContent = `${fmt(rg[0])}…${fmt(rg[1])} cm`;
  $('pitch').textContent = fmt(t.pitch_deg, 1);
  chart('pitchChart', ['p'], ['#81a273'], [], null);
  $('pitchHint').textContent = !isNum(t.pitch_deg) ? 'Waiting for an angle reading' : Math.abs(t.pitch_deg) > 30 ? 'Beyond 30°: tilt fault' : 'Hump starts above +5°';
  $('distance').textContent = fmt(t.dist_mm);
  $('encL').textContent = fmt(t.enc?.[0]); $('encR').textContent = fmt(t.enc?.[1]);
  $('lineError').textContent = fmt(t.line?.err);
  $('lineLost').textContent = t.line?.lost === undefined ? '—' : t.line.lost ? 'Lost' : 'On line';

  /* obstacle */
  const ob = t.obstacle || {};
  $('width').textContent = fmt(ob.w_mm); $('clrL').textContent = fmt(ob.clr_l_cm); $('clrR').textContent = fmt(ob.clr_r_cm);
  const prof = [...(snap.events || [])].reverse().find(e => isType(e, 'PROFILE_DONE'));
  $('obsAge').textContent = prof ? 'PROFILED ' + clock(prof.time) : 'NO SCAN YET';
  const tw = Number($('trueWidth').value);
  if (isNum(ob.w_mm) && tw > 0) {
    const err = Math.abs(ob.w_mm - tw) / tw * 100, b = band(err, [10, 20, 30]);
    $('widthGrade').className = 'grade ' + b;
    $('widthGrade').innerHTML = `<b>${fmt(err, 1)} % error</b>${bandText(b)} (${fmt(ob.w_mm)} vs ${fmt(tw)} mm)`;
  } else { $('widthGrade').className = 'grade'; $('widthGrade').textContent = isNum(ob.w_mm) ? 'Enter the box width to see the error' : 'No width measured yet'; }
  const br = bypassResult();
  if (br && isNum(br.sec)) {
    const b = br.end === 'RECOVERY_DONE' ? band(br.sec, [2.999, 4.999, 6.999]) : 'f';
    $('bypassTime').innerHTML = `<span class="band-${b}">${fmt(br.sec, 2)} s · ${br.end === 'RECOVERY_DONE' ? bandText(b) : 'not completed'}</span>`;
  } else $('bypassTime').textContent = br ? 'in progress…' : '—';
  $('bypassResult').textContent = br ? br.end.replaceAll('_', ' ').toLowerCase() : '—';
  renderPath();

  /* tables */
  const sc = scans();
  $('scanCount').textContent = sc.length + ' SCANS';
  $('scanRows').innerHTML = sc.length ? sc.slice(-60).reverse().map(r => `<tr><td>${esc(clock(r.t))}</td><td class="sym">${esc(r.c)}</td><td><span class="tag ${r.tone}">${esc(r.res)}</span></td></tr>`).join('')
    : '<tr><td colspan="3" class="muted">No barcodes yet.</td></tr>';
  const hu = humps();
  $('humpMax').textContent = 'HIGHEST ' + (isNum(t.hump_max_mm) && t.hump_max_mm > 0 ? t.hump_max_mm + ' MM' : hu.length ? Math.max(...hu.map(h => h.peak)) + ' MM' : '—');
  $('humpRows').innerHTML = hu.length ? hu.slice(-40).reverse().map(h => `<tr><td>${esc(clock(h.t))}</td><td>${esc(fmt(h.peak))} mm</td><td>${isNum(h.pitch) ? esc(fmt(h.pitch, 1)) + '°' : '—'}</td><td>${isNum(h.dur) ? esc(fmt(h.dur, 2)) + ' s' : '—'}</td></tr>`).join('')
    : '<tr><td colspan="4" class="muted">No humps yet.</td></tr>';

  $('footerMode').textContent = mode === 'demo' ? 'Demo mode · no commands sent' : mode === 'local' ? `Broker ${snap.host || '127.0.0.1'}:1883` : 'No robot connected';
  renderEvents();
}

function renderPath() {
  const ep = snap.bypass, el = $('pathView');
  if (!ep || !ep.samples?.length) {
    el.innerHTML = '<text x="200" y="135" text-anchor="middle" font-size="12" fill="#9aa292">No bypass yet</text>';
    $('pathLabel').textContent = 'LAST BYPASS'; return;
  }
  const pts = pathPoints(ep), d = isNum(ep.d_cm) ? ep.d_cm * 10 : 200, w = isNum(ep.w_mm) ? ep.w_mm : 150;
  const xs = pts.map(p => p[0]).concat([0, d + w]), ys = pts.map(p => p[1]).concat([-w / 2 - 60, w / 2 + 60]);
  const minX = Math.min(...xs) - 40, maxX = Math.max(...xs) + 40, minY = Math.min(...ys) - 30, maxY = Math.max(...ys) + 30;
  const sc = Math.min(380 / (maxX - minX), 240 / (maxY - minY));
  /* forward = right on screen, left of the robot = up */
  const X = x => 10 + (x - minX) * sc, Y = y => 250 - (y - minY) * sc;
  let svg = `<line x1="0" x2="400" y1="${Y(0)}" y2="${Y(0)}" stroke="#2a3a33" stroke-width="${Math.max(2, 18 * sc)}" opacity=".85"/>`;
  svg += `<rect x="${X(d)}" y="${Y(w / 2)}" width="${Math.max(4, w * sc)}" height="${w * sc}" fill="#e7c7a3" stroke="#a8662d" rx="3"/>`;
  svg += `<text x="${X(d) + w * sc / 2}" y="${Y(w / 2) - 6}" text-anchor="middle" font-size="10" fill="#8a5a2a">${esc(fmt(w))} mm</text>`;
  svg += `<path d="${pts.map((p, i) => (i ? 'L' : 'M') + X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)).join('')}" fill="none" stroke="#3f8a5e" stroke-width="2.5" stroke-linejoin="round"/>`;
  svg += `<circle cx="${X(0)}" cy="${Y(0)}" r="4" fill="#3f8a5e"/>`;
  const last = pts[pts.length - 1];
  svg += `<circle cx="${X(last[0])}" cy="${Y(last[1])}" r="4" fill="#fff" stroke="#3f8a5e" stroke-width="2"/>`;
  const maxDev = Math.max(...pts.map(p => Math.abs(p[1])));
  svg += `<text x="8" y="16" font-size="10" fill="#6c7a6e">max sideways ${esc(fmt(maxDev))} mm · ${esc(fmt(last[0]))} mm forward</text>`;
  el.innerHTML = svg;
  $('pathLabel').textContent = ep.done ? 'LAST BYPASS · ' + clock(ep.start) : 'BYPASS IN PROGRESS';
}

function renderEvents() {
  const all = snap.events || [];
  const noisy = e => isLog(e, 100) || isLog(e, 101) || isLog(e, 102) || isLog(e, 103) || isLog(e, 104);
  const list = all.filter(e => filter === 'diagnostic' ? describe(e).cat === 'diagnostic' : !noisy(e) && (filter === 'all' || describe(e).cat === filter));
  $('eventCount').textContent = all.length; $('eventTotal').textContent = all.length + ' EVENTS';
  const shown = list.slice(-150).reverse(), sig = mode + filter + shown.map(e => e.key).join(',') + (snap.commands || []).map(c => c.result).join();
  if (sig === lastEventSig) return;
  lastEventSig = sig;
  $('eventList').innerHTML = shown.length ? shown.map(e => {
    const d = describe(e);
    return `<div class="event-row"><span class="event-symbol ${d.tone}">${d.sym}</span><div><div class="event-title">${esc(d.title)}</div><div class="event-detail">${esc(d.detail)}</div></div><time class="event-time">${esc(clock(e.time))}</time></div>`;
  }).join('') : `<div class="empty"><span>≋</span><b>${all.length ? 'No matching events.' : 'Nothing yet.'}</b><p>${all.length ? 'Choose another filter.' : 'State changes, faults, barcodes and acknowledgements will appear here.'}</p></div>`;
}

/* ---------- local server ---------- */
function notice(title, text) { $('notice').querySelector('b').textContent = title; $('notice').querySelector(':scope > div > span').textContent = text; }
async function api(path, body) {
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Dashboard-Token': token}, body: JSON.stringify(body)});
  const x = await r.json(); if (!r.ok) throw Error(x.error || 'Request failed'); return x;
}
async function poll() {
  if (pollBusy || mode === 'demo') return;
  pollBusy = true;
  try {
    const r = await fetch('/api/status', {cache: 'no-store'});
    if (!r.ok) throw Error('Not a local server');
    const d = await r.json(); if (d.mode !== 'local') throw Error('Not local');
    if (mode !== 'local') notice('Local dashboard is running.', 'Keep this laptop on the Pico’s hotspot. Telemetry appears when the robot connects.');
    mode = 'local'; d.fetched = Date.now() / 1000; snap = d; token = d.token;
    $('localConnection').hidden = false; $('connectBtn').textContent = '⚙ Connection';
    if (d.error) notice('Check the connection.', d.error);
    render();
  } catch (e) {
    if (mode === 'local') { snap.broker = false; snap.error = 'Lost the local dashboard. Check the launcher window.'; render(); }
  } finally { pollBusy = false; }
}
async function send(cmd, args = {}) {
  if (mode !== 'local') { toast(mode === 'demo' ? 'Demo mode: nothing is sent.' : 'Not connected.'); return; }
  try {
    const c = await api('/api/command', {cmd, args});
    toast(c.result === 'publish failed' ? 'Publish failed: ' + (c.error || 'check the broker') : `${c.label} sent. Waiting for the robot's acknowledgement.`);
    await poll();
  } catch (e) { toast(e.message); }
}

/* ---------- demo: a simulated run with a barcode, a bypass and a hump ---------- */
function demoStart() {
  demo = {k: 0, seq: 0, tms: 0, dist: 0, el: 0, er: 0};
  snap = {telemetry: null, status: {online: true, rssi: -48, drops: 0, uptime_s: 0, map: {left: 'A', right: 'F', straight: 'Z', uturn: 'W'}},
    received: 0, history: [], events: [], commands: [], bypass: null};
  mode = 'demo'; $('demoBtn').textContent = 'Exit demo';
  notice('You’re exploring a demo.', 'All values are simulated: a barcode, a junction turn, an obstacle bypass and a hump. Nothing is sent to a robot.');
  demo.timer = setInterval(demoTick, 200); demoTick();
}
function demoEvent(type, data, extra = {}) { snap.events.push({key: ++demo.k, time: Date.now() / 1000, topic: 'picocar/is23/event', payload: Object.assign({seq: demo.k, t_ms: demo.tms, type, data, src: 5}, extra)}); }
function demoTick() {
  const T = demo.seq * 0.2 % 40, now = Date.now() / 1000;
  let st = 'LINE_FOLLOW', vl = 250, vr = 250, d = 120, pitch = 0.3 * Math.sin(demo.seq / 4), w = null, nav = null;
  const once = (key, fn) => { if (demo[key] !== Math.floor(demo.seq * 0.2 / 40)) { demo[key] = Math.floor(demo.seq * 0.2 / 40); fn(); } };
  if (T < 2) { st = 'IDLE'; vl = vr = 0; once('e0', () => demoEvent('STATE', 'IDLE')); }
  else if (T < 6) { once('e1', () => { demoEvent('STATE', 'LINE_FOLLOW'); }); vl = 250 + 25 * Math.sin(T * 3); vr = 250 - 25 * Math.sin(T * 3); }
  else if (T < 9) { nav = 'A'; once('e2', () => { demoEvent('BARCODE', 'A'); demoEvent('LOG', (0 << 16) | 65, {code: 63}); }); }
  else if (T < 11) { st = 'EXECUTE_NAV'; nav = 'A'; once('e3', () => { demoEvent('JUNCTION', 0); demoEvent('STATE', 'EXECUTE_NAV'); }); vl = -130; vr = 130; }
  else if (T < 16) { once('e4', () => demoEvent('STATE', 'LINE_FOLLOW')); d = 120 - (T - 11) * 20; if (d < 40) { vl = vr = 150; once('e5', () => demoEvent('SLOW_DOWN', Math.round(d * 10))); } }
  else if (T < 17.6) { st = 'OBSTACLE_HANDLING'; vl = vr = 0; d = 19; w = 148; once('e6', () => { demoEvent('OBSTACLE', 190); demoEvent('STATE', 'OBSTACLE_HANDLING'); }); if (T > 17.2) once('e7', () => demoEvent('PROFILE_DONE', 148)); }
  else if (T < 21.6) { st = 'AVOIDING'; w = 148; d = null; once('e8', () => demoEvent('STATE', 'AVOIDING'));
    /* turn out 60 deg (2 ticks at +-170 mm/s), offset 200 mm, turn back, pass 300 mm, turn in, return 200 mm */
    const u = T - 17.6, legs = [[0.4, -170, 170], [0.8, 250, 250], [0.4, 170, -170], [1.2, 250, 250], [0.4, 170, -170], [0.8, 250, 250]];
    let acc = 0; [vl, vr] = [250, 250]; for (const [len, l, r] of legs) { if (u < acc + len - 1e-6) { [vl, vr] = [l, r]; break; } acc += len; } }
  else if (T < 22.6) { st = 'LINE_RECOVERY'; w = 148; [vl, vr] = T < 21.99 ? [-170, 170] : [150, 150]; once('e9', () => demoEvent('STATE', 'LINE_RECOVERY')); if (T > 22.3) once('e10', () => { demoEvent('RECOVERY_DONE', 0); demoEvent('STATE', 'LINE_FOLLOW'); }); }
  else if (T < 24) { w = 148; }
  else if (T < 30) { w = 148; if (T > 26 && T < 28) { pitch = 9 * Math.sin((T - 26) / 2 * Math.PI); once('e11', () => demoEvent('HUMP_START', 0)); } if (T > 28.2) once('e12', () => demoEvent('HUMP_END', 23)); }
  else if (T < 34) { w = 148; if (T > 31) once('e13', () => { demoEvent('BARCODE', 'Q'); demoEvent('LOG', 81, {code: 64}); }); }
  else { st = 'MISSION_COMPLETE'; vl = vr = 0; w = 148; once('e14', () => { demoEvent('STATE', 'MISSION_COMPLETE'); demoEvent('LOG', 23, {code: 67}); }); }
  demo.seq++; demo.tms += 200; demo.dist += (vl + vr) / 2 * 0.2; demo.el += vl * 0.2 / MM_PER_COUNT; demo.er += vr * 0.2 / MM_PER_COUNT;
  const t = {seq: demo.seq, t_ms: demo.tms, state: st, v_mm_s: [Math.round(vl), Math.round(vr)], enc: [Math.round(demo.el), Math.round(demo.er)],
    dist_mm: Math.round(demo.dist), line: {err: Math.round((vl - vr) * 4), lost: false}, barcode: T > 6 ? (T > 31 ? 'Q' : 'A') : null,
    pitch_deg: Math.round(pitch * 10) / 10, hump_max_mm: T > 28.2 ? 23 : 0,
    obstacle: {d_cm: d === null ? null : Math.round(d * 10) / 10, w_cm: w && Math.round(w / 10), w_mm: w, clr_l_cm: w && 32, clr_r_cm: w && 28}, nav};
  snap.telemetry = t; snap.received = now; snap.status.uptime_s = Math.floor(demo.seq / 5);
  snap.history.push({t: now, vl: t.v_mm_s[0], vr: t.v_mm_s[1], d: t.obstacle.d_cm, p: t.pitch_deg, st});
  snap.history = snap.history.filter(x => now - x.t <= 120);
  if (BYPASS_STATES.includes(st)) {
    if (!snap.bypass || snap.bypass.done) snap.bypass = {start: now, done: false, d_cm: 19, w_mm: 148, samples: []};
    snap.bypass.samples.push({tms: t.t_ms, vl: t.v_mm_s[0], vr: t.v_mm_s[1], el: t.enc[0], er: t.enc[1], st});
  } else if (snap.bypass && !snap.bypass.done) snap.bypass.done = true;
  if (snap.events.length > 600) snap.events = snap.events.slice(-600);
  render();
}
function demoStop() { clearInterval(demo?.timer); demo = null; mode = 'offline'; snap = {}; $('demoBtn').textContent = 'Try demo'; notice('Ready when your robot is.', 'Start the local dashboard with Start-Dashboard.cmd, or try the demo.'); render(); poll(); }

/* ---------- wiring ---------- */
$('demoBtn').onclick = () => mode === 'demo' ? demoStop() : demoStart();
['connectBtn', 'setupNav', 'getStarted'].forEach(id => $(id).onclick = () => $('setup').showModal());
document.querySelectorAll('[data-jump]').forEach(b => b.onclick = () => {
  document.querySelectorAll('.nav').forEach(x => x.classList.toggle('selected', x === b));
  b.dataset.jump === 'top' ? window.scrollTo({top: 0, behavior: 'smooth'}) : $(b.dataset.jump).scrollIntoView({behavior: 'smooth', block: 'start'});
});
$('applyConnect').onclick = async () => { try { await api('/api/connect', {host: $('brokerHost').value.trim()}); $('setupError').textContent = ''; $('setup').close(); await poll(); } catch (e) { $('setupError').textContent = e.message; } };
$('disconnect').onclick = async () => { try { await api('/api/disconnect', {}); await poll(); toast('Disconnected. The broker and robot keep running.'); } catch (e) { toast(e.message); } };
$('estop').onclick = () => send('estop');
$('stop').onclick = () => send('stop');
$('reset').onclick = () => $('resetDialog').showModal();
$('cancelReset').onclick = () => $('resetDialog').close();
$('confirmReset').onclick = () => { $('resetDialog').close(); send('reset'); };
$('start').onclick = () => {
  const m = snap.status?.map;
  $('startCheck').textContent = 'The robot will drive off along the line. ' + (m ? `Map: ${m.left}=Left, ${m.right}=Right, ${m.straight}=Straight, ${m.uturn}=U-turn.` : 'No barcode map is set: barcodes will be read but ignored.')
    + ' A start before calibration (GP21 tap) is refused with not_calibrated.';
  $('startDialog').showModal();
};
$('cancelStart').onclick = () => $('startDialog').close();
$('confirmStart').onclick = () => { $('startDialog').close(); send('start'); };
$('mapForm').onsubmit = e => {
  e.preventDefault();
  const f = new FormData($('mapForm')), args = {};
  for (const k of ['left', 'right', 'straight', 'uturn']) args[k] = String(f.get(k) || '').trim().toUpperCase();
  if (new Set(Object.values(args)).size !== 4 || Object.values(args).some(v => v.length !== 1)) { toast('Enter four different characters.'); return; }
  send('map', args);
};
$('mapForm').querySelectorAll('input').forEach((inp, i, all) => inp.addEventListener('input', () => { inp.value = inp.value.toUpperCase(); if (inp.value && all[i + 1]) all[i + 1].focus(); }));
document.querySelectorAll('[data-bench]').forEach(b => b.onclick = () => send('bench', {param: b.dataset.bench, value: Number(b.dataset.value)}));
$('trueWidth').addEventListener('input', render);
document.querySelectorAll('[data-filter]').forEach(b => b.onclick = () => { filter = b.dataset.filter; document.querySelectorAll('[data-filter]').forEach(x => x.classList.toggle('active', x === b)); lastEventSig = ''; renderEvents(); });
document.addEventListener('keydown', e => {
  if (e.repeat || e.ctrlKey || e.altKey || e.metaKey || ['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName) || document.querySelector('dialog[open]')) return;
  if (e.key.toLowerCase() === 'e' && !$('estop').disabled) { e.preventDefault(); send('estop'); }
  if (e.key === ' ' && !$('stop').disabled) { e.preventDefault(); send('stop'); }
});
setInterval(() => { $('clock').textContent = new Date().toLocaleTimeString([], {hour: '2-digit', minute: '2-digit'}); if (mode === 'local') render(); }, 1000);
setInterval(poll, 500);
render(); poll();
