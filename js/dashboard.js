/* ==========================================================================
   CGR-1 Console
   Runs a self-contained simulation of the robot (map, A* planner, LiDAR,
   camera + landmark detection, voice/intent, TTS) so the dashboard works
   without hardware. The ROBOT adapter at the top is where the real robot
   plugs in (rosbridge / Flask endpoints).
   ========================================================================== */

const CONFIG = {
  simSpeed: 2,            // sim time multiplier
  intentEndpoint: null,   // e.g. '/api/intent'  -> POST {text, lang} returns {target, response}
  rosbridgeUrl: null,     // e.g. 'ws://turtlebot.local:9090' (wire up in ROBOT below)
};

/* --------------------------------------------------------------------------
   ROBOT adapter. Replace the bodies with roslibjs / fetch calls when the
   robot is online. The sim calls these so the log shows what would be sent.
   -------------------------------------------------------------------------- */
const ROBOT = {
  sendGoal(x, y, theta) {
    // e.g. new ROSLIB.ActionClient(... '/navigate_to_pose' ...)
    log('nav2', `NavigateToPose  x=${x.toFixed(2)} y=${y.toFixed(2)} θ=${theta.toFixed(2)}`);
  },
  cancelGoal() { log('nav2', 'goal canceled'); },
  cmdVel(v, w) { /* publish geometry_msgs/Twist on /cmd_vel */ },
  estop(on) { log('safety', on ? 'E-STOP engaged → /cmd_vel = 0, goals locked' : 'E-STOP released'); },
};

/* ==========================================================================
   World: 4.0 m × 3.0 m cardboard campus, 0.05 m occupancy grid
   Internal frame: x right, y down (screen). Displayed in ROS /map frame
   (y up) as y_map = WORLD_H - y, θ_map = -θ.
   ========================================================================== */
const WORLD_W = 4.0, WORLD_H = 3.0, RES = 0.05;
const GW = Math.round(WORLD_W / RES), GH = Math.round(WORLD_H / RES);
const ROBOT_R = 0.105, INFLATE = 0.16;
const VMAX = 0.22, WMAX = 1.6;
const DOCK = { x: 0.55, y: 1.40, th: 0 };

const B = {
  library:     { code: 'B-01', slug: 'library',          en: 'Main Library',     ko: '중앙 도서관', rect: [0.35, 0.35, 1.15, 0.85], h: 0.18, goal: [0.75, 1.12, -Math.PI / 2],
                 keys: ['library', 'book', 'study', '도서관', '책'],
                 about_en: 'Books, study rooms and quiet reading areas.', about_ko: '책과 열람실, 조용한 학습 공간이 있습니다.' },
  union:       { code: 'B-02', slug: 'student_union',    en: 'Student Union',    ko: '학생회관',   rect: [2.55, 0.30, 3.60, 0.75], h: 0.17, goal: [3.05, 1.02, -Math.PI / 2],
                 keys: ['union', 'student center', 'club', '학생회관', '학생회'],
                 about_en: 'Student clubs, events and the council office.', about_ko: '동아리와 행사, 학생회 사무실이 있습니다.' },
  cafe:        { code: 'B-03', slug: 'cafe',             en: 'Campus Cafe',      ko: '캠퍼스 카페', rect: [1.55, 0.95, 2.20, 1.35], h: 0.15, goal: [1.88, 1.62, -Math.PI / 2],
                 keys: ['cafe', 'coffee', 'food', 'eat', '카페', '커피'],
                 about_en: 'Coffee, snacks and a place to sit down.', about_ko: '커피와 간식을 즐길 수 있습니다.' },
  engineering: { code: 'B-04', slug: 'engineering_dept', en: 'Engineering Bldg', ko: '공학관',     rect: [2.85, 1.65, 3.60, 2.35], h: 0.19, goal: [2.58, 2.00, 0],
                 keys: ['engineering', 'lab', '공학관', '공대'],
                 about_en: 'Labs, workshops and engineering classrooms.', about_ko: '실험실과 공학 강의실이 있습니다.' },
  business:    { code: 'B-05', slug: 'business_dept',    en: 'Business School',  ko: '경영대학',   rect: [1.40, 2.35, 2.20, 2.75], h: 0.18, goal: [1.80, 2.08, Math.PI / 2],
                 keys: ['business', 'management', 'economics', '경영', '경영대학'],
                 about_en: 'Business and management lecture halls.', about_ko: '경영학 강의실이 있습니다.' },
  restroom:    { code: 'B-06', slug: 'restroom',         en: 'Restrooms',        ko: '화장실',     rect: [0.30, 1.90, 0.85, 2.45], h: 0.14, goal: [1.12, 2.18, Math.PI],
                 keys: ['restroom', 'toilet', 'bathroom', 'wc', '화장실'],
                 about_en: 'Restrooms are right here.', about_ko: '화장실은 바로 이곳입니다.' },
};
const PILLARS = [{ x: 1.30, y: 1.48, r: 0.05 }];

/* ---------- occupancy + inflation ---------- */
const occ = new Uint8Array(GW * GH);
const cost = new Uint8Array(GW * GH);
(function buildGrid() {
  for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) {
    const x = (i + 0.5) * RES, y = (j + 0.5) * RES;
    let o = i === 0 || j === 0 || i === GW - 1 || j === GH - 1;
    for (const k in B) { const r = B[k].rect; if (x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3]) o = true; }
    for (const p of PILLARS) if (Math.hypot(x - p.x, y - p.y) <= p.r + RES / 2) o = true;
    occ[j * GW + i] = o ? 1 : 0;
  }
  const rc = Math.ceil(INFLATE / RES);
  for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) {
    if (!occ[j * GW + i]) continue;
    for (let dj = -rc; dj <= rc; dj++) for (let di = -rc; di <= rc; di++) {
      const ii = i + di, jj = j + dj;
      if (ii < 0 || jj < 0 || ii >= GW || jj >= GH) continue;
      if (Math.hypot(di, dj) * RES <= INFLATE) cost[jj * GW + ii] = 1;
    }
  }
})();
const cellOf = (x, y) => [Math.min(GW - 1, Math.max(0, Math.floor(x / RES))), Math.min(GH - 1, Math.max(0, Math.floor(y / RES)))];
const isOcc = (x, y) => { const [i, j] = cellOf(x, y); return occ[j * GW + i] === 1; };
const isCost = (x, y) => { const [i, j] = cellOf(x, y); return cost[j * GW + i] === 1; };

/* ---------- A* planner (8-connected, octile) ---------- */
function plan(sx, sy, gx, gy) {
  const [si, sj] = cellOf(sx, sy);
  let [gi, gj] = cellOf(gx, gy);
  if (cost[gj * GW + gi]) {                       // snap goal to nearest free cell
    let best = null, bd = 1e9;
    for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) {
      if (cost[j * GW + i]) continue;
      const d = (i - gi) ** 2 + (j - gj) ** 2;
      if (d < bd) { bd = d; best = [i, j]; }
    }
    if (!best) return null; [gi, gj] = best;
  }
  const N = GW * GH, g = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
  const heap = [];
  const push = (n, f) => { heap.push([f, n]); let c = heap.length - 1; while (c > 0) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, r = l + 1; let m = c; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
  const h = (i, j) => { const dx = Math.abs(i - gi), dy = Math.abs(j - gj); return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy); };
  const s = sj * GW + si, goal = gj * GW + gi;
  g[s] = 0; push(s, h(si, sj));
  const D = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  while (heap.length) {
    const [, n] = pop();
    if (closed[n]) continue; closed[n] = 1;
    if (n === goal) break;
    const i = n % GW, j = (n / GW) | 0;
    for (const [di, dj, c] of D) {
      const ii = i + di, jj = j + dj;
      if (ii < 0 || jj < 0 || ii >= GW || jj >= GH) continue;
      const m = jj * GW + ii;
      if (closed[m]) continue;
      if (occ[m]) continue;
      // never enter inflation, except to escape when the robot already starts inside it
      if (cost[m] && m !== goal && (!cost[s] || Math.hypot(ii - si, jj - sj) > INFLATE / RES + 1)) continue;
      if (di && dj && occ[j * GW + ii] && occ[jj * GW + i]) continue;
      const ng = g[n] + c;
      if (ng < g[m]) { g[m] = ng; came[m] = n; push(m, ng + h(ii, jj)); }
    }
  }
  if (came[goal] === -1 && goal !== s) return null;
  const cells = []; for (let n = goal; n !== -1; n = came[n]) cells.push(n);
  cells.reverse();
  let pts = cells.map(n => ({ x: (n % GW + 0.5) * RES, y: (((n / GW) | 0) + 0.5) * RES }));
  pts[0] = { x: sx, y: sy };
  const [ogi, ogj] = cellOf(gx, gy);
  if (gi === ogi && gj === ogj) pts[pts.length - 1] = { x: gx, y: gy };
  return smooth(pts);
}
function lineFree(a, b) {
  const d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.ceil(d / (RES / 3));
  for (let k = 1; k < n; k++) { const t = k / n; if (isCost(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false; }
  return true;
}
function smooth(pts) {
  if (pts.length < 3) return pts;
  const out = [pts[0]]; let i = 0;
  while (i < pts.length - 1) {
    let j = pts.length - 1;
    while (j > i + 1 && !lineFree(pts[i], pts[j])) j--;
    out.push(pts[j]); i = j;
  }
  return out;
}
const pathLen = p => p.reduce((s, q, i) => i ? s + Math.hypot(q.x - p[i - 1].x, q.y - p[i - 1].y) : 0, 0);
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

/* ==========================================================================
   State
   ========================================================================== */
const S = {
  lang: 'en',
  mode: 'AUTO',
  estop: false,
  pose: { x: DOCK.x, y: DOCK.y, th: DOCK.th },
  v: 0, w: 0,
  nav: null,                // { key, name, goal:[x,y,th], path, wp, total, phase }
  trail: [],
  layers: { scan: true, cost: true, trail: true },
  scan: [],                 // [{a, r}]
  scanMin: null,
  battDrive: 85, battAux: 92,
  held: null,               // teleop direction
  t0: performance.now(),
  hover: null,
  listening: false,
  lastDet: null,
};

/* ==========================================================================
   i18n
   ========================================================================== */
const T = {
  en: {
    console: 'Console', map: 'Map · Nav2', camera: 'Camera · YOLOv8', request: 'Request · intent', tft: 'Onboard TFT',
    drive: 'Manual drive', log: 'Event log', clear: 'CLEAR', send: 'SEND', dock: 'DOCK',
    map_hint: 'click map → 2D goal pose', drive_hint: 'Hold to drive. Cancels the active goal.',
    placeholder: 'Where is the library?', listening: 'Listening…',
    tft_ready: ['Hello!', 'Ask me where a building is'], tft_go: n => [n, 'Follow me'], tft_arrived: n => [n, 'We have arrived'],
    tft_manual: ['Manual drive', 'Operator in control'], tft_stop: ['STOP', 'Emergency stop active'],
    say_go: n => `Follow me to the ${n}.`, say_arrive: (n, a) => `We have arrived at the ${n}. ${a}`,
    say_unknown: "Sorry, I don't know that place yet. Try the library, cafe or business school.",
    say_estop: 'Emergency stop activated.', say_blocked: 'Emergency stop is active. Release it first.',
    say_lang: 'Language set to English.', say_noplan: 'I could not find a path there.',
    chips: ['Where is the Business School?', 'Take me to the cafe', 'I need a restroom', '도서관 어디에 있나요?'],
    custom: 'Custom goal', searching: 'searching…', idle: 'idle',
  },
  ko: {
    console: '콘솔', map: '지도 · Nav2', camera: '카메라 · YOLOv8', request: '요청 · 의도 분석', tft: '로봇 화면',
    drive: '수동 조종', log: '이벤트 로그', clear: '지우기', send: '전송', dock: '복귀',
    map_hint: '지도 클릭 → 목표 지정', drive_hint: '누르고 있으면 이동합니다. 현재 목표는 취소됩니다.',
    placeholder: '도서관 어디에 있나요?', listening: '듣고 있습니다…',
    tft_ready: ['안녕하세요!', '가고 싶은 건물을 물어보세요'], tft_go: n => [n, '저를 따라오세요'], tft_arrived: n => [n, '도착했습니다'],
    tft_manual: ['수동 조종', '운영자 제어 중'], tft_stop: ['정지', '비상 정지 작동 중'],
    say_go: n => `${n}(으)로 안내하겠습니다. 저를 따라오세요.`, say_arrive: (n, a) => `${n}에 도착했습니다. ${a}`,
    say_unknown: '죄송합니다, 아직 모르는 장소입니다. 도서관, 카페, 경영대학 등을 말씀해 주세요.',
    say_estop: '비상 정지가 작동되었습니다.', say_blocked: '비상 정지 상태입니다. 먼저 해제해 주세요.',
    say_lang: '한국어로 설정되었습니다.', say_noplan: '그곳으로 가는 경로를 찾지 못했습니다.',
    chips: ['경영대학 어디에 있나요?', '카페로 데려다 주세요', '화장실이 어디예요?', 'Where is the library?'],
    custom: '사용자 지정 목표', searching: '탐색 중…', idle: '대기',
  },
};
const t = k => T[S.lang][k];
const bname = b => (S.lang === 'ko' ? b.ko : b.en);

function setLang(lang, silent) {
  S.lang = lang;
  document.documentElement.lang = lang;
  document.getElementById('lang-en').classList.toggle('on', lang === 'en');
  document.getElementById('lang-ko').classList.toggle('on', lang === 'ko');
  document.querySelectorAll('[data-i18n]').forEach(el => { const v = T[lang][el.dataset.i18n]; if (typeof v === 'string') el.textContent = v; });
  document.getElementById('ask-text').placeholder = t('placeholder');
  renderChips(); renderDests(); refreshTFT();
  if (!silent) { speak(t('say_lang')); log('sys', `ui language → ${lang}`); }
}

/* ==========================================================================
   Log
   ========================================================================== */
const logEl = document.getElementById('log');
function log(src, msg) {
  const ts = ((performance.now() - S.t0) / 1000).toFixed(3).padStart(8, ' ');
  const row = document.createElement('div');
  row.innerHTML = `<span class="t">[${ts}]</span><span class="src ${src}">${src}</span>`;
  row.appendChild(document.createTextNode(msg));
  logEl.appendChild(row);
  while (logEl.children.length > 300) logEl.firstChild.remove();
  logEl.scrollTop = logEl.scrollHeight;
}
function clearLog() { logEl.innerHTML = ''; }

/* ==========================================================================
   Destinations + chips
   ========================================================================== */
function renderDests() {
  const el = document.getElementById('dests');
  el.innerHTML = '';
  for (const k in B) {
    const b = B[k];
    const btn = document.createElement('button');
    btn.className = 'dest' + (S.nav && S.nav.key === k ? ' active' : '');
    btn.innerHTML = `<span class="code">${b.code}</span><span class="name"></span><span class="xy">${b.goal[0].toFixed(2)}, ${(WORLD_H - b.goal[1]).toFixed(2)}</span>`;
    btn.querySelector('.name').textContent = bname(b);
    btn.onclick = () => goTo(k);
    el.appendChild(btn);
  }
}
function renderChips() {
  const el = document.getElementById('chips');
  el.innerHTML = '';
  t('chips').forEach(q => {
    const b = document.createElement('button');
    b.textContent = q; b.onclick = () => handleText(q);
    el.appendChild(b);
  });
}

/* ==========================================================================
   Navigation
   ========================================================================== */
function goTo(key, custom) {
  if (S.estop) { speak(t('say_blocked')); log('safety', 'goal rejected: e-stop active'); return false; }
  if (S.mode === 'MANUAL') { document.getElementById('manual').checked = false; setManual(false); }
  const b = key ? B[key] : null;
  const goal = b ? b.goal : custom;
  const path = plan(S.pose.x, S.pose.y, goal[0], goal[1]);
  if (!path) { log('nav2', 'planner failed: no valid path'); speak(t('say_noplan')); setNavState('FAILED', 'err'); return false; }
  const end = path[path.length - 1];
  S.nav = { key, name: b ? null : 'custom', goal: [end.x, end.y, goal[2]], path, wp: 1, total: pathLen(path), phase: 'DRIVE' };
  ROBOT.sendGoal(end.x, WORLD_H - end.y, -goal[2]);
  log('nav2', `global plan: ${path.length} waypoints, ${S.nav.total.toFixed(2)} m`);
  setNavState('NAVIGATING', 'active');
  renderDests(); refreshTFT();
  return true;
}
function cancelNav(quiet) {
  if (!S.nav) return;
  S.nav = null; S.v = 0; S.w = 0;
  if (!quiet) ROBOT.cancelGoal();
  setNavState('IDLE'); renderDests(); refreshTFT();
}
function dock() {
  if (S.estop) return;
  goTo(null, [DOCK.x, DOCK.y, DOCK.th]);
}
function setNavState(txt, cls) {
  const el = document.getElementById('nav-state');
  el.textContent = txt; el.className = cls || '';
}

/* map click → custom goal */
const mapCanvas = document.getElementById('map');
mapCanvas.addEventListener('mousemove', e => { S.hover = mapEvt(e); });
mapCanvas.addEventListener('mouseleave', () => { S.hover = null; });
mapCanvas.addEventListener('click', e => {
  const p = mapEvt(e);
  if (isCost(p.x, p.y)) { log('nav2', `goal (${p.x.toFixed(2)}, ${(WORLD_H - p.y).toFixed(2)}) is in collision, rejected`); return; }
  const th = Math.atan2(p.y - S.pose.y, p.x - S.pose.x);
  goTo(null, [p.x, p.y, th]);
});
function mapEvt(e) {
  const r = mapCanvas.getBoundingClientRect();
  return { x: (e.clientX - r.left) / r.width * WORLD_W, y: (e.clientY - r.top) / r.height * WORLD_H };
}

/* ==========================================================================
   Intent parsing (local fallback for the LLM)
   ========================================================================== */
function parseLocal(text) {
  const q = text.toLowerCase();
  for (const k in B) if (B[k].keys.some(w => q.includes(w))) return { key: k, target: B[k].slug };
  return { key: null, target: null };
}
async function handleText(text) {
  text = text.trim(); if (!text) return;
  const isKo = /[가-힣]/.test(text);
  if (isKo !== (S.lang === 'ko')) setLang(isKo ? 'ko' : 'en', true);
  log('llm', `user: "${text}"`);
  const t1 = performance.now();
  let res;
  if (CONFIG.intentEndpoint) {
    try {
      const r = await fetch(CONFIG.intentEndpoint, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ text, lang: S.lang }) });
      res = await r.json();
      res.key = Object.keys(B).find(k => B[k].slug === res.target) || null;
    } catch (e) { log('llm', 'endpoint unreachable, using local parser'); }
  }
  if (!res) {
    const p = parseLocal(text);
    res = { key: p.key, target: p.target, response: p.key ? t('say_go')(bname(B[p.key])) : t('say_unknown') };
  }
  const ms = Math.round(performance.now() - t1 + (CONFIG.intentEndpoint ? 0 : 120 + Math.random() * 90));
  document.getElementById('parse-ms').textContent = `${ms} ms · ${CONFIG.intentEndpoint ? 'api' : 'local'}`;
  showJSON({ target: res.target, response: res.response });
  log('llm', `→ target=${res.target ?? 'null'}`);
  speak(res.response);
  if (res.key) goTo(res.key);
}
function showJSON(o) {
  const esc = s => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
  const val = v => v === null ? '<span class="jn">null</span>' : `<span class="js">"${esc(v)}"</span>`;
  document.getElementById('json').innerHTML =
    `{\n  <span class="jk">"target"</span>: ${val(o.target)},\n  <span class="jk">"response"</span>: ${val(o.response)}\n}`;
}
function submitText() {
  const el = document.getElementById('ask-text');
  handleText(el.value); el.value = '';
}

/* ---------- speech in ---------- */
let rec = null;
(function initSpeech() {
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SR) return;
  rec = new SR(); rec.continuous = false; rec.interimResults = false;
  rec.onstart = () => { S.listening = true; document.getElementById('mic').classList.add('live'); document.getElementById('ask-text').placeholder = t('listening'); };
  rec.onresult = e => handleText(e.results[0][0].transcript);
  rec.onend = rec.onerror = () => { S.listening = false; document.getElementById('mic').classList.remove('live'); document.getElementById('ask-text').placeholder = t('placeholder'); };
})();
function toggleMic() {
  if (!rec) { log('sys', 'Web Speech API not available in this browser; type instead'); document.getElementById('ask-text').focus(); return; }
  if (S.listening) rec.stop();
  else { rec.lang = S.lang === 'ko' ? 'ko-KR' : 'en-US'; try { rec.start(); } catch (e) {} }
}

/* ---------- speech out ---------- */
function speak(text) {
  const spk = document.getElementById('speaker');
  log('tts', `"${text}"`);
  spk.textContent = 'SPK ▶ PLAYING';
  const done = () => { spk.textContent = 'SPK IDLE'; };
  if ('speechSynthesis' in window) {
    speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    u.lang = S.lang === 'ko' ? 'ko-KR' : 'en-US';
    u.onend = done; u.onerror = done;
    speechSynthesis.speak(u);
    setTimeout(done, 200 + text.length * 70);
  } else setTimeout(done, 2000);
}

/* ==========================================================================
   Safety + teleop
   ========================================================================== */
function toggleEstop() {
  S.estop = !S.estop;
  document.body.classList.toggle('estopped', S.estop);
  document.getElementById('estop-text').textContent = S.estop ? 'RELEASE' : 'E-STOP';
  ROBOT.estop(S.estop);
  if (S.estop) {
    S.v = 0; S.w = 0; ROBOT.cmdVel(0, 0);
    if (S.nav) { S.nav = null; renderDests(); }
    setNavState('E-STOP', 'err');
    speak(t('say_estop'));
  } else setNavState('IDLE');
  refreshTFT();
}
function setManual(on) {
  S.mode = on ? 'MANUAL' : 'AUTO';
  document.getElementById('mode-text').textContent = S.mode;
  document.getElementById('manual-label').textContent = on ? 'ON' : 'OFF';
  document.getElementById('pad').classList.toggle('disabled', !on);
  if (on) { cancelNav(); log('teleop', 'manual override ON, autonomy paused'); }
  else { S.held = null; S.v = 0; S.w = 0; log('teleop', 'manual override OFF'); }
  refreshTFT();
}
document.querySelectorAll('#pad button').forEach(btn => {
  const dir = btn.dataset.dir;
  const start = e => { e.preventDefault(); if (dir === 'stop') { S.held = null; S.v = S.w = 0; return; } S.held = dir; btn.classList.add('held'); };
  const end = () => { if (S.held === dir) S.held = null; btn.classList.remove('held'); };
  btn.addEventListener('pointerdown', start);
  btn.addEventListener('pointerup', end); btn.addEventListener('pointerleave', end); btn.addEventListener('pointercancel', end);
});
const KEYMAP = { w: 'up', arrowup: 'up', s: 'down', arrowdown: 'down', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
const keysDown = new Set();
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  const k = e.key.toLowerCase();
  if (k === 'escape') { if (!S.estop) toggleEstop(); return; }
  if (S.mode !== 'MANUAL') return;
  if (k === ' ') { e.preventDefault(); S.held = null; keysDown.clear(); return; }
  if (KEYMAP[k]) { e.preventDefault(); keysDown.add(KEYMAP[k]); S.held = KEYMAP[k]; highlightPad(); }
});
window.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (KEYMAP[k]) { keysDown.delete(KEYMAP[k]); S.held = [...keysDown].pop() || null; highlightPad(); }
});
function highlightPad() { document.querySelectorAll('#pad button').forEach(b => b.classList.toggle('held', b.dataset.dir === S.held)); }

/* ==========================================================================
   TFT mirror
   ========================================================================== */
function refreshTFT() {
  let lines = t('tft_ready'), state = 'READY', alert = false;
  if (S.estop) { lines = t('tft_stop'); state = 'E-STOP'; alert = true; }
  else if (S.mode === 'MANUAL') { lines = t('tft_manual'); state = 'MANUAL'; }
  else if (S.nav) {
    const n = S.nav.key ? bname(B[S.nav.key]) : t('custom');
    lines = S.nav.phase === 'DONE' ? t('tft_arrived')(n) : t('tft_go')(n);
    state = S.nav.phase === 'DONE' ? 'ARRIVED' : 'MOVING';
  }
  document.getElementById('tft-main').textContent = lines[0];
  document.getElementById('tft-sub').textContent = lines[1];
  document.getElementById('tft-state').textContent = state;
  document.getElementById('tft-mode').textContent = S.estop ? 'STOP' : S.mode;
  document.querySelector('.tft').classList.toggle('alert', alert);
}

/* ==========================================================================
   Simulation step
   ========================================================================== */
function step(dt) {
  const P = S.pose;
  if (S.estop) { S.v = 0; S.w = 0; }
  else if (S.mode === 'MANUAL') {
    const d = S.held;
    S.v = d === 'up' ? VMAX : d === 'down' ? -VMAX * 0.6 : 0;
    S.w = d === 'left' ? -1.2 : d === 'right' ? 1.2 : 0;   // screen frame: y down
  } else if (S.nav && S.nav.phase !== 'DONE') {
    const N = S.nav;
    if (N.phase === 'DRIVE') {
      const tgt = N.path[N.wp];
      const d = Math.hypot(tgt.x - P.x, tgt.y - P.y);
      const last = N.wp === N.path.length - 1;
      if (d < (last ? 0.025 : 0.07)) {
        if (last) { N.phase = 'ALIGN'; S.v = 0; }
        else N.wp++;
      } else {
        const e = wrap(Math.atan2(tgt.y - P.y, tgt.x - P.x) - P.th);
        if (Math.abs(e) > 0.6) { S.v = 0; S.w = Math.sign(e) * 1.4; }
        else { S.v = Math.min(VMAX, last ? 0.05 + d * 1.2 : VMAX) * Math.cos(e); S.w = Math.max(-WMAX, Math.min(WMAX, 2.6 * e)); }
      }
    }
    if (N.phase === 'ALIGN') {
      const e = wrap(N.goal[2] - P.th);
      if (Math.abs(e) < 0.03) { arrive(); S.v = 0; S.w = 0; }
      else { S.v = 0; S.w = Math.max(-1.2, Math.min(1.2, 3 * e)); if (Math.abs(S.w) < 0.25) S.w = 0.25 * Math.sign(e); }
    }
  } else { S.v = 0; S.w = 0; }

  // integrate unicycle
  const nth = wrap(P.th + S.w * dt);
  const nx = P.x + S.v * Math.cos(nth) * dt, ny = P.y + S.v * Math.sin(nth) * dt;
  let blocked = false;
  for (let a = 0; a < Math.PI * 2; a += Math.PI / 8) if (isOcc(nx + Math.cos(a) * ROBOT_R, ny + Math.sin(a) * ROBOT_R)) { blocked = true; break; }
  P.th = nth;
  if (!blocked) { P.x = nx; P.y = ny; }
  else if (S.v !== 0) { if (!S._bumpT || performance.now() - S._bumpT > 1500) { log('safety', 'collision guard: motion blocked'); S._bumpT = performance.now(); } S.v = 0; }
  ROBOT.cmdVel(S.v, -S.w);

  // trail
  const lt = S.trail[S.trail.length - 1];
  if (!lt || Math.hypot(lt.x - P.x, lt.y - P.y) > 0.02) { S.trail.push({ x: P.x, y: P.y }); if (S.trail.length > 600) S.trail.shift(); }

  // battery drain (simulated)
  S.battDrive = Math.max(0, S.battDrive - (Math.abs(S.v) * 0.02 + Math.abs(S.w) * 0.004) * dt);
  S.battAux = Math.max(0, S.battAux - 0.0008 * dt);
}
function arrive() {
  const N = S.nav; N.phase = 'DONE';
  setNavState('SUCCEEDED', 'ok');
  if (N.key) {
    const b = B[N.key];
    log('yolo', `landmark lock ${b.slug}: h/H ≥ 0.62 → arrived`);
    speak(t('say_arrive')(bname(b), S.lang === 'ko' ? b.about_ko : b.about_en));
  } else log('nav2', 'goal reached');
  refreshTFT();
  setTimeout(() => { if (S.nav === N) { S.nav = null; setNavState('IDLE'); renderDests(); refreshTFT(); } }, 6000);
}

/* ---------- LiDAR (LDS: 360 samples, 0.12–3.5 m; we sample every 2°) ---------- */
function scan() {
  const P = S.pose, out = []; let min = null;
  for (let k = 0; k < 180; k++) {
    const a = k * Math.PI / 90;
    const ga = P.th + a;
    const c = Math.cos(ga), s = Math.sin(ga);
    let r = 0.12, hit = false;
    while (r < 3.5) { if (isOcc(P.x + c * r, P.y + s * r)) { hit = true; break; } r += 0.01; }
    if (hit) {
      r += (Math.random() - 0.5) * 0.012;
      out.push({ a, r });
      if (!min || r < min.r) min = { a, r };
    }
  }
  S.scan = out; S.scanMin = min;
}

/* ==========================================================================
   Map rendering
   ========================================================================== */
const mctx = mapCanvas.getContext('2d');
let staticLayer = null, MS = 1; // px per meter

function sizeMap() {
  const r = mapCanvas.getBoundingClientRect(), dpr = Math.min(2, window.devicePixelRatio || 1);
  mapCanvas.width = Math.round(r.width * dpr); mapCanvas.height = Math.round(r.height * dpr);
  MS = mapCanvas.width / WORLD_W;
  buildStatic();
}
function buildStatic() {
  const c = document.createElement('canvas'); c.width = mapCanvas.width; c.height = mapCanvas.height;
  const g = c.getContext('2d'), cs = RES * MS;
  g.fillStyle = '#0C0D0F'; g.fillRect(0, 0, c.width, c.height);
  // costmap inflation
  if (S.layers.cost) {
    g.fillStyle = '#21140D';   // opaque, pre-blended: avoids seams between cells
    for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) if (cost[j * GW + i] && !occ[j * GW + i]) g.fillRect(i * cs, j * cs, cs + 0.5, cs + 0.5);
  }
  // grid 0.25 m minor / 1 m major
  for (let x = 0; x <= WORLD_W + 1e-6; x += 0.25) {
    g.strokeStyle = Math.abs(x % 1) < 1e-6 ? 'rgba(255,255,255,0.09)' : 'rgba(255,255,255,0.035)';
    g.beginPath(); g.moveTo(x * MS + 0.5, 0); g.lineTo(x * MS + 0.5, c.height); g.stroke();
  }
  for (let y = 0; y <= WORLD_H + 1e-6; y += 0.25) {
    g.strokeStyle = Math.abs(y % 1) < 1e-6 ? 'rgba(255,255,255,0.09)' : 'rgba(255,255,255,0.035)';
    g.beginPath(); g.moveTo(0, y * MS + 0.5); g.lineTo(c.width, y * MS + 0.5); g.stroke();
  }
  // occupied cells
  g.fillStyle = '#C9C8C3';
  for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) if (occ[j * GW + i]) g.fillRect(i * cs, j * cs, cs + 0.5, cs + 0.5);
  // building labels
  const fs = Math.max(9, MS * 0.045);
  g.font = `500 ${fs}px "IBM Plex Mono", monospace`; g.textBaseline = 'top';
  for (const k in B) {
    const b = B[k], r = b.rect;
    g.fillStyle = '#16171A';
    g.fillText(b.code, r[0] * MS + 6, r[1] * MS + 6);
    g.fillText(b.slug, r[0] * MS + 6, r[1] * MS + 6 + fs * 1.3);
  }
  // goal markers
  for (const k in B) {
    const [gx, gy, gt] = B[k].goal;
    g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1;
    const x = gx * MS, y = gy * MS, s = MS * 0.035;
    g.beginPath(); g.moveTo(x - s, y); g.lineTo(x + s, y); g.moveTo(x, y - s); g.lineTo(x, y + s); g.stroke();
  }
  // dock
  g.strokeStyle = 'rgba(255,255,255,0.3)'; g.setLineDash([3, 3]);
  g.strokeRect((DOCK.x - 0.13) * MS, (DOCK.y - 0.13) * MS, 0.26 * MS, 0.26 * MS); g.setLineDash([]);
  g.fillStyle = 'rgba(255,255,255,0.35)'; g.font = `${fs * 0.9}px "IBM Plex Mono", monospace`;
  g.fillText('DOCK', (DOCK.x - 0.13) * MS, (DOCK.y + 0.15) * MS);
  staticLayer = c;
}
function drawMap() {
  const g = mctx, P = S.pose;
  g.drawImage(staticLayer, 0, 0);
  const px = P.x * MS, py = P.y * MS;
  // trail
  if (S.layers.trail && S.trail.length > 1) {
    g.strokeStyle = 'rgba(255,255,255,0.18)'; g.lineWidth = Math.max(1, MS * 0.006);
    g.beginPath(); S.trail.forEach((p, i) => i ? g.lineTo(p.x * MS, p.y * MS) : g.moveTo(p.x * MS, p.y * MS)); g.stroke();
  }
  // plan
  if (S.nav) {
    const path = S.nav.path;
    g.strokeStyle = '#FF4F00'; g.lineWidth = Math.max(1.5, MS * 0.01); g.setLineDash([MS * 0.03, MS * 0.02]);
    g.beginPath(); g.moveTo(px, py);
    for (let i = S.nav.wp; i < path.length; i++) g.lineTo(path[i].x * MS, path[i].y * MS);
    g.stroke(); g.setLineDash([]);
    const [gx, gy, gt] = S.nav.goal;
    g.strokeStyle = '#FF4F00'; g.lineWidth = 2;
    g.beginPath(); g.arc(gx * MS, gy * MS, MS * 0.04, 0, Math.PI * 2); g.stroke();
    g.beginPath(); g.moveTo(gx * MS, gy * MS); g.lineTo((gx + Math.cos(gt) * 0.09) * MS, (gy + Math.sin(gt) * 0.09) * MS); g.stroke();
  }
  // scan
  if (S.layers.scan) {
    g.fillStyle = '#3FCF7F';
    const ds = Math.max(2, MS * 0.009);
    for (const s of S.scan) {
      const a = P.th + s.a;
      g.fillRect(px + Math.cos(a) * s.r * MS - ds / 2, py + Math.sin(a) * s.r * MS - ds / 2, ds, ds);
    }
    // camera FOV wedge
    const f = 31.1 * Math.PI / 180;
    g.fillStyle = 'rgba(127,180,255,0.06)'; g.strokeStyle = 'rgba(127,180,255,0.25)'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(px, py); g.arc(px, py, MS * 0.9, P.th - f, P.th + f); g.closePath(); g.fill(); g.stroke();
  }
  // robot
  const rr = ROBOT_R * MS;
  g.fillStyle = S.estop ? '#E5372B' : '#E6E5E1';
  g.beginPath(); g.arc(px, py, rr, 0, Math.PI * 2); g.fill();
  g.strokeStyle = '#0A0B0C'; g.lineWidth = Math.max(1.5, MS * 0.008);
  g.beginPath(); g.moveTo(px, py); g.lineTo(px + Math.cos(P.th) * rr, py + Math.sin(P.th) * rr); g.stroke();
  g.fillStyle = '#FF4F00'; g.beginPath(); g.arc(px, py, rr * 0.32, 0, Math.PI * 2); g.fill();
  // scale bar
  const sx = 14 * (mapCanvas.width / mapCanvas.clientWidth), sy = mapCanvas.height - 16 * (mapCanvas.width / mapCanvas.clientWidth);
  g.strokeStyle = '#E6E5E1'; g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(sx, sy - 5); g.lineTo(sx, sy); g.lineTo(sx + 0.5 * MS, sy); g.lineTo(sx + 0.5 * MS, sy - 5); g.stroke();
  g.fillStyle = '#E6E5E1'; g.font = `${Math.max(10, MS * 0.04)}px "IBM Plex Mono", monospace`; g.textBaseline = 'bottom';
  g.fillText('0.5 m', sx + 6, sy - 4);
  // hover cursor
  const hud = document.getElementById('map-cursor');
  hud.textContent = S.hover ? `x ${S.hover.x.toFixed(2)}  y ${(WORLD_H - S.hover.y).toFixed(2)}  ${isCost(S.hover.x, S.hover.y) ? '· lethal' : '· free'}` : 'campus.pgm · 80×60 · 0.05 m/px';
}

/* ==========================================================================
   Camera rendering + landmark detection
   ========================================================================== */
const cam = document.getElementById('cam'), cctx = cam.getContext('2d');
const CW = 640, CH = 360, HFOV = 62.2 * Math.PI / 180, FX = (CW / 2) / Math.tan(HFOV / 2), HZ = CH * 0.46, CAM_H = 0.10;

function drawCam(now) {
  const g = cctx, P = S.pose;
  // sky / floor
  g.fillStyle = '#121316'; g.fillRect(0, 0, CW, HZ);
  const fl = g.createLinearGradient(0, HZ, 0, CH); fl.addColorStop(0, '#1C1D20'); fl.addColorStop(1, '#2A2B2F');
  g.fillStyle = fl; g.fillRect(0, HZ, CW, CH - HZ);
  // floor grid lines (perspective, world-anchored)
  g.strokeStyle = 'rgba(255,255,255,0.05)'; g.lineWidth = 1;
  for (let k = 1; k < 12; k++) { const d = k * 0.25, y = HZ + FX * CAM_H / d; if (y < CH) { g.beginPath(); g.moveTo(0, y); g.lineTo(CW, y); g.stroke(); } }

  // project buildings
  const vis = [];
  for (const k in B) {
    const b = B[k], r = b.rect;
    const corners = [[r[0], r[1]], [r[2], r[1]], [r[2], r[3]], [r[0], r[3]]];
    let minX = Infinity, maxX = -Infinity, minDepth = Infinity, behind = 0;
    for (const [cx, cy] of corners) {
      const dx = cx - P.x, dy = cy - P.y;
      const fwd = dx * Math.cos(P.th) + dy * Math.sin(P.th);
      const lat = -dx * Math.sin(P.th) + dy * Math.cos(P.th);
      if (fwd < 0.05) { behind++; continue; }
      const u = CW / 2 + FX * lat / fwd;
      minX = Math.min(minX, u); maxX = Math.max(maxX, u); minDepth = Math.min(minDepth, fwd);
    }
    if (behind === 4 || maxX < 0 || minX > CW || !isFinite(minDepth)) continue;
    if (behind > 0) { // partially behind: extend to the frame edge on the side that wraps
      const dx = (r[0] + r[2]) / 2 - P.x, dy = (r[1] + r[3]) / 2 - P.y;
      const lat = -dx * Math.sin(P.th) + dy * Math.cos(P.th);
      if (lat < 0) minX = -50; else maxX = CW + 50;
    }
    const top = HZ - FX * (b.h - CAM_H) / minDepth, bot = HZ + FX * CAM_H / minDepth;
    vis.push({ k, b, l: minX, r: maxX, t: top, bt: bot, d: minDepth });
  }
  vis.sort((a, b) => b.d - a.d);
  for (const v of vis) {
    const w = v.r - v.l, h = v.bt - v.t;
    g.fillStyle = '#3B3D42'; g.fillRect(v.l, v.t, w, h);
    g.fillStyle = '#46484E'; g.fillRect(v.l, v.t, w, Math.max(2, h * 0.07));
    // windows
    g.fillStyle = '#1A1B1E';
    const cols = Math.max(2, Math.round(w / Math.max(18, h * 0.22))), rows = 3;
    const cw = w / (cols * 2 + 1), rh = h / (rows * 2 + 2);
    if (cw > 2 && rh > 2) for (let i = 0; i < cols; i++) for (let j = 0; j < rows; j++) g.fillRect(v.l + cw * (1 + i * 2), v.t + rh * (1.5 + j * 2), cw, rh);
  }

  // detection
  const target = S.nav && S.nav.key ? S.nav.key : null;
  let det = null;
  for (const v of vis) {
    const l = Math.max(0, v.l), r = Math.min(CW, v.r), tp = Math.max(0, v.t), bt = Math.min(CH, v.bt);
    if (r - l < 14 || bt - tp < 18) continue;
    const clipped = (v.l < 0 || v.r > CW) ? 0.12 : 0;
    const conf = Math.max(0.35, Math.min(0.97, 0.95 - clipped - Math.max(0, v.d - 1.6) * 0.18 + Math.sin(now / 260 + v.d * 9) * 0.015));
    const isT = v.k === target || (!target && false);
    g.lineWidth = isT ? 2.5 : 1;
    g.strokeStyle = isT ? '#FF4F00' : 'rgba(127,180,255,0.75)';
    g.strokeRect(l + 0.5, tp + 0.5, r - l - 1, bt - tp - 1);
    const label = `${v.b.slug} ${conf.toFixed(2)}`;
    g.font = `500 ${isT ? 12 : 10.5}px "IBM Plex Mono", monospace`;
    const tw = g.measureText(label).width + 10, th = isT ? 18 : 15;
    const ly = tp > th ? tp - th : tp;
    g.fillStyle = isT ? '#FF4F00' : 'rgba(127,180,255,0.85)'; g.fillRect(l, ly, tw, th);
    g.fillStyle = isT ? '#fff' : '#0A0B0C'; g.textBaseline = 'middle'; g.fillText(label, l + 5, ly + th / 2 + 1);
    if (isT) det = { slug: v.b.slug, conf, ex: (l + r) / 2 - CW / 2, h: (bt - tp) / CH, cx: (l + r) / 2, tp, bt };
  }

  // HUD
  g.setLineDash([5, 5]); g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1;
  g.beginPath(); g.moveTo(CW / 2 + 0.5, 0); g.lineTo(CW / 2 + 0.5, CH); g.stroke(); g.setLineDash([]);
  if (det) {
    g.strokeStyle = '#fff'; g.lineWidth = 1.2;
    g.beginPath(); g.moveTo(CW / 2, 24); g.lineTo(det.cx, 24); g.stroke();
    g.beginPath(); g.moveTo(det.cx - Math.sign(det.ex) * 6, 20); g.lineTo(det.cx, 24); g.lineTo(det.cx - Math.sign(det.ex) * 6, 28); g.stroke();
    // h gauge
    const gx = CW - 18, gh = CH - 40;
    g.strokeStyle = 'rgba(255,255,255,0.35)'; g.strokeRect(gx, 20, 8, gh);
    g.fillStyle = det.h >= 0.62 ? '#3FCF7F' : '#FF4F00';
    const fh = Math.min(1, det.h) * gh; g.fillRect(gx, 20 + gh - fh, 8, fh);
    g.fillStyle = '#fff'; g.fillRect(gx - 5, 20 + gh * (1 - 0.62), 18, 1.5);
  }
  // corner brackets
  g.strokeStyle = 'rgba(255,255,255,0.5)'; g.lineWidth = 1.5;
  [[10, 10, 1, 1], [CW - 10, 10, -1, 1], [10, CH - 10, 1, -1], [CW - 10, CH - 10, -1, -1]].forEach(([x, y, sx, sy]) => {
    g.beginPath(); g.moveTo(x, y + 16 * sy); g.lineTo(x, y); g.lineTo(x + 16 * sx, y); g.stroke();
  });
  g.font = '500 10.5px "IBM Plex Mono", monospace'; g.textBaseline = 'alphabetic'; g.fillStyle = 'rgba(255,255,255,0.65)';
  g.fillText(`REC ● ${((now - S.t0) / 1000).toFixed(1)}s`, 18, CH - 16);
  g.fillText('YOLOv8n · imgsz 640', CW - 160, CH - 16);
  if (S.estop) { g.fillStyle = 'rgba(229,55,43,0.18)'; g.fillRect(0, 0, CW, CH); }

  // readouts
  const set = (id, v) => { const el = document.getElementById(id); if (el.textContent !== v) el.textContent = v; };
  if (det) {
    set('cv-target', det.slug); set('cv-conf', det.conf.toFixed(2));
    set('cv-ex', `${det.ex >= 0 ? '+' : ''}${det.ex.toFixed(0)} px`);
    set('cv-h', `${det.h.toFixed(2)}${det.h >= 0.62 ? ' ✓' : ''}`);
    set('cv-steer', Math.abs(det.ex) < 12 ? 'CENTER' : det.ex < 0 ? `◀ ${(-det.ex / CW * 1.2).toFixed(2)}` : `${(det.ex / CW * 1.2).toFixed(2)} ▶`);
  } else {
    set('cv-target', target ? B[target].slug : '—'); set('cv-conf', '—'); set('cv-ex', '—'); set('cv-h', '—');
    set('cv-steer', target ? t('searching') : t('idle'));
  }
  if (det && (!S.lastDet || S.lastDet !== det.slug)) log('yolo', `acquired ${det.slug} conf=${det.conf.toFixed(2)}`);
  S.lastDet = det ? det.slug : null;
}

/* ==========================================================================
   Telemetry
   ========================================================================== */
function telemetry() {
  const P = S.pose, set = (id, v) => { const el = document.getElementById(id); if (el.textContent !== v) el.textContent = v; };
  set('pose-x', P.x.toFixed(2)); set('pose-y', (WORLD_H - P.y).toFixed(2));
  set('pose-t', `${Math.round((-P.th * 180 / Math.PI + 360) % 360)}°`);
  set('cv-v', S.v.toFixed(2)); set('cv-w', (-S.w).toFixed(2));
  set('batt-drive', S.battDrive.toFixed(0)); set('batt-aux', S.battAux.toFixed(0));
  document.getElementById('batt-drive-bar').style.width = S.battDrive + '%';
  document.getElementById('batt-aux-bar').style.width = S.battAux + '%';
  set('tft-batt', S.battDrive.toFixed(0) + '%');
  if (S.nav) {
    const N = S.nav;
    let rem = Math.hypot(N.path[Math.min(N.wp, N.path.length - 1)].x - P.x, N.path[Math.min(N.wp, N.path.length - 1)].y - P.y);
    for (let i = N.wp; i < N.path.length - 1; i++) rem += Math.hypot(N.path[i + 1].x - N.path[i].x, N.path[i + 1].y - N.path[i].y);
    if (N.phase !== 'DRIVE') rem = 0;
    const pct = Math.max(0, Math.min(100, (1 - rem / N.total) * 100));
    set('route-rem', rem.toFixed(2));
    document.getElementById('route-bar').style.width = pct + '%';
    document.getElementById('tft-prog').style.width = pct + '%';
    set('nav-goal', (N.key ? B[N.key].slug : 'custom') + ` · ETA ${(rem / VMAX / CONFIG.simSpeed).toFixed(0)}s`);
  } else {
    set('route-rem', '—'); set('nav-goal', 'no goal');
    document.getElementById('route-bar').style.width = '0%';
    document.getElementById('tft-prog').style.width = '0%';
  }
  if (S.scanMin) {
    set('scan-min', S.scanMin.r.toFixed(2));
    const deg = Math.round((-S.scanMin.a * 180 / Math.PI + 360) % 360);
    set('scan-bearing', `@ ${deg}° · ${S.scan.length}/180 returns`);
  }
  const s = Math.floor((performance.now() - S.t0) / 1000);
  set('uptime', [s / 3600, (s / 60) % 60, s % 60].map(n => String(Math.floor(n)).padStart(2, '0')).join(':'));
}

/* ==========================================================================
   Layers / loop / boot
   ========================================================================== */
function toggleLayer(k) {
  S.layers[k] = !S.layers[k];
  document.getElementById('t-' + k).classList.toggle('on', S.layers[k]);
  if (k === 'cost') buildStatic();
}

let last = performance.now(), lastScan = 0, lastTel = 0, frames = 0, fpsT = performance.now();
function loop(now) {
  const dt = Math.min(0.05, (now - last) / 1000) * CONFIG.simSpeed; last = now;
  step(dt);
  if (now - lastScan > 100) { scan(); lastScan = now; }
  drawMap();
  drawCam(now);
  if (now - lastTel > 100) { telemetry(); lastTel = now; }
  frames++; if (now - fpsT > 1000) { document.getElementById('fps').textContent = frames; frames = 0; fpsT = now; }
  requestAnimationFrame(loop);
}

window.addEventListener('resize', () => { clearTimeout(window._rz); window._rz = setTimeout(sizeMap, 120); });
document.fonts && document.fonts.ready.then(() => buildStatic());

(function boot() {
  setLang('en', true);
  sizeMap();
  scan();
  log('sys', 'console up · simulation mode (no rosbridge configured)');
  log('sys', 'map loaded: campus.pgm 80×60 @ 0.05 m');
  log('nav2', 'lifecycle: planner_server, controller_server active');
  log('yolo', 'model loaded: yolov8n-campus.pt (6 classes)');
  requestAnimationFrame(loop);
})();
