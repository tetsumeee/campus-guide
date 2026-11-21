/* ==========================================================================
   CGR-1 Console (demo)
   A self-contained simulation of the robot: campus map with routing,
   LiDAR, camera + landmark detection, the Gil voice assistant and TTS.
   Nothing here talks to the real robot; the ROBOT adapter below is where
   rosbridge calls would go later.
   ========================================================================== */

const CONFIG = {
  simSpeed: 1,            // 1 = real time at 0.22 m/s
  intentEndpoint: null,   // set automatically to '/api/ask' when served by server/app.py
  knowledgeUrl: 'data/campus.json',
};

// Robot hooks. These do nothing in the demo; js/robot.js replaces them when the
// robot is switched on in js/robot-config.js and rosbridge is reachable.
const ROBOT = {
  live: () => false,                 // true while connected: pose comes from the robot
  step(dt) {},                       // called instead of the simulation step while live
  sendGoal(key, x, y, th) { log('nav2', `goal ${key} x=${x.toFixed(2)} y=${(WORLD_H - y).toFixed(2)}`); },
  cancelGoal() {},
  estop(on) {},
  tft(info) {},                      // mirror the TFT text to the robot's screen
  say(text, lang) {},                // also speak on the robot's speaker
  drawCamera(g, w, h) { return false; },
};

/* ==========================================================================
   World: 4.0 m × 3.0 m campus model, 0.05 m grid. Frame: x right, y down.
   Shown to users in ROS /map frame (y up): y_map = WORLD_H - y.
   ========================================================================== */
const WORLD_W = 4.0, WORLD_H = 3.0, RES = 0.05;
const GW = Math.round(WORLD_W / RES), GH = Math.round(WORLD_H / RES);
const ROBOT_R = 0.105, INFLATE = 0.16;
const VMAX = 0.22, WMAX = 1.6;
const DOCK = { x: 0.60, y: 1.58, th: -Math.PI / 2 };

// Material icon paths (Apache 2.0), 24×24 viewBox
const ICON = {
  library: 'M12 11.55C9.64 9.35 6.48 8 3 8v11c3.48 0 6.64 1.35 9 3.55 2.36-2.19 5.52-3.55 9-3.55V8c-3.48 0-6.64 1.35-9 3.55zM12 8c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3z',
  union: 'M16 11c1.66 0 2.99-1.34 2.99-3S17.66 5 16 5c-1.66 0-3 1.34-3 3s1.34 3 3 3zm-8 0c1.66 0 2.99-1.34 2.99-3S9.66 5 8 5C6.34 5 5 6.34 5 8s1.34 3 3 3zm0 2c-2.33 0-7 1.17-7 3.5V19h14v-2.5c0-2.33-4.67-3.5-7-3.5zm8 0c-.29 0-.62.02-.97.05 1.16.84 1.97 1.97 1.97 3.45V19h6v-2.5c0-2.33-4.67-3.5-7-3.5z',
  cafe: 'M20 3H4v10c0 2.21 1.79 4 4 4h6c2.21 0 4-1.79 4-4v-3h2c1.11 0 2-.9 2-2V5c0-1.11-.89-2-2-2zm0 5h-2V5h2v3zM4 19h16v2H4z',
  engineering: 'M22.7 19l-9.1-9.1c.9-2.3.4-5-1.5-6.9-2-2-5-2.4-7.4-1.3L9 6 6 9 1.6 4.7C.4 7.1.9 10.1 2.9 12.1c1.9 1.9 4.6 2.4 6.9 1.5l9.1 9.1c.4.4 1 .4 1.4 0l2.3-2.3c.5-.4.5-1.1.1-1.4z',
  business: 'M20 6h-4V4c0-1.11-.89-2-2-2h-4c-1.11 0-2 .89-2 2v2H4c-1.11 0-1.99.89-1.99 2L2 19c0 1.11.89 2 2 2h16c1.11 0 2-.89 2-2V8c0-1.11-.89-2-2-2zm-6 0h-4V4h4v2z',
  restroom: 'M5.5 22v-7.5H4V9c0-1.1.9-2 2-2h3c1.1 0 2 .9 2 2v5.5H9.5V22h-4zM18 22v-6h3l-2.54-7.63C18.18 7.55 17.42 7 16.56 7h-.12c-.86 0-1.63.55-1.9 1.37L12 16h3v6h3zM7.5 6c1.11 0 2-.89 2-2s-.89-2-2-2-2 .89-2 2 .89 2 2 2zm9 0c1.11 0 2-.89 2-2s-.89-2-2-2-2 .89-2 2 .89 2 2 2z',
  pin: 'M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z',
  left: 'M6.83 11l1.59 1.59L7 14 3 10l4-4 1.41 1.41L6.83 9H15c2.21 0 4 1.79 4 4v7h-2v-7c0-1.1-.9-2-2-2H6.83z',
  right: 'M17.17 11l-1.59 1.59L17 14l4-4-4-4-1.41 1.41L17.17 9H9c-2.21 0-4 1.79-4 4v7h2v-7c0-1.1.9-2 2-2h8.17z',
  straight: 'M11 6.83L9.41 8.41 8 7l4-4 4 4-1.41 1.41L13 6.83V21h-2z',
};

const B = {
  library:     { code: 'B-01', slug: 'library',          en: 'Main Library',     ko: '중앙 도서관', cat_en: 'Library',        cat_ko: '도서관',   color: '#4E6AD4',
                 rect: [0.30, 0.30, 1.05, 0.85], h: 0.18, goal: [0.68, 1.10, -Math.PI / 2], keys: ['library', 'book', 'study', '도서관', '책'] },
  cafe:        { code: 'B-02', slug: 'cafe',             en: 'Campus Cafe',      ko: '캠퍼스 카페', cat_en: 'Coffee shop',    cat_ko: '카페',     color: '#E37400',
                 rect: [1.62, 0.42, 2.13, 0.85], h: 0.15, goal: [1.875, 1.10, -Math.PI / 2], keys: ['cafe', 'café', 'coffee', 'food', 'eat', '카페', '커피'] },
  union:       { code: 'B-03', slug: 'student_union',    en: 'Student Union',    ko: '학생회관',   cat_en: 'Student center', cat_ko: '학생 시설', color: '#12A4AF',
                 rect: [2.75, 0.28, 3.65, 0.85], h: 0.17, goal: [3.20, 1.10, -Math.PI / 2], keys: ['union', 'student center', 'club', '학생회관', '학생회'] },
  engineering: { code: 'B-04', slug: 'engineering_dept', en: 'Engineering Bldg', ko: '공학관',     cat_en: 'University building', cat_ko: '대학 건물', color: '#8E63CE',
                 rect: [2.75, 1.30, 3.65, 1.82], h: 0.19, goal: [3.20, 2.05, -Math.PI / 2], keys: ['engineering', 'lab', '공학관', '공대'] },
  business:    { code: 'B-05', slug: 'business_dept',    en: 'Business School',  ko: '경영대학',   cat_en: 'University building', cat_ko: '대학 건물', color: '#B06000',
                 rect: [1.55, 2.30, 2.20, 2.78], h: 0.18, goal: [1.875, 2.05, Math.PI / 2], keys: ['business', 'management', 'economics', '경영', '경영대학'] },
  restroom:    { code: 'B-06', slug: 'restroom',         en: 'Restrooms',        ko: '화장실',     cat_en: 'Restroom',       cat_ko: '화장실',   color: '#5F6368',
                 rect: [0.38, 2.30, 0.95, 2.70], h: 0.14, goal: [0.665, 2.05, Math.PI / 2], keys: ['restroom', 'toilet', 'bathroom', 'wc', '화장실'] },
};
const ROADS = [
  { en: 'University Ave', ko: '대학로',     a: [0.10, 1.10], b: [3.90, 1.10], w: 0.13 },
  { en: 'College Rd',     ko: '학교길',     a: [0.10, 2.05], b: [3.90, 2.05], w: 0.13 },
  { en: 'Library Ln',     ko: '도서관길',   a: [1.30, 0.10], b: [1.30, 2.90], w: 0.13 },
  { en: 'Union St',       ko: '학생회관길', a: [2.45, 0.10], b: [2.45, 2.90], w: 0.13 },
  { en: 'Dock path',      ko: '도크 길',    a: [0.60, 1.10], b: [0.60, 1.58], w: 0.08, minor: true },
];
const PLAZA = { rect: [1.42, 1.22, 2.33, 1.93], en: 'Central Plaza', ko: '중앙 광장' };
const FOUNTAIN = { x: 1.875, y: 1.575, r: 0.08 };
const LAKE = { x: 3.20, y: 2.52, rx: 0.42, ry: 0.20, en: 'Lake', ko: '호수' };
const XS = [0.05, 1.30, 2.45, 3.95], YS = [0.05, 1.10, 2.05, 2.95];   // block edges (road centre lines)

/* ---------- occupancy, inflation, road mask ---------- */
const occ = new Uint8Array(GW * GH), cost = new Uint8Array(GW * GH), road = new Uint8Array(GW * GH);
const inRect = (x, y, r) => x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3];
function segDist(px, py, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const t = Math.max(0, Math.min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / (dx * dx + dy * dy)));
  return Math.hypot(px - a[0] - t * dx, py - a[1] - t * dy);
}
(function buildGrid() {
  for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) {
    const x = (i + 0.5) * RES, y = (j + 0.5) * RES, n = j * GW + i;
    let o = i === 0 || j === 0 || i === GW - 1 || j === GH - 1;
    for (const k in B) if (inRect(x, y, B[k].rect)) o = true;
    if (Math.hypot(x - FOUNTAIN.x, y - FOUNTAIN.y) <= FOUNTAIN.r + RES / 2) o = true;
    if (((x - LAKE.x) / LAKE.rx) ** 2 + ((y - LAKE.y) / LAKE.ry) ** 2 <= 1) o = true;
    occ[n] = o ? 1 : 0;
    road[n] = ROADS.some(r => segDist(x, y, r.a, r.b) <= r.w / 2 + 0.005) || inRect(x, y, PLAZA.rect) ? 1 : 0;
  }
  const rc = Math.ceil(INFLATE / RES);
  for (let j = 0; j < GH; j++) for (let i = 0; i < GW; i++) {
    if (!occ[j * GW + i]) continue;
    for (let dj = -rc; dj <= rc; dj++) for (let di = -rc; di <= rc; di++) {
      const ii = i + di, jj = j + dj;
      if (ii >= 0 && jj >= 0 && ii < GW && jj < GH && Math.hypot(di, dj) * RES <= INFLATE) cost[jj * GW + ii] = 1;
    }
  }
})();
const cellOf = (x, y) => [Math.min(GW - 1, Math.max(0, Math.floor(x / RES))), Math.min(GH - 1, Math.max(0, Math.floor(y / RES)))];
const idx = (x, y) => { const [i, j] = cellOf(x, y); return j * GW + i; };
const isOcc = (x, y) => occ[idx(x, y)] === 1;
const isCost = (x, y) => cost[idx(x, y)] === 1;
const isRoad = (x, y) => road[idx(x, y)] === 1;

/* ---------- A* planner: walkways are cheap, lawns are expensive ---------- */
const OFFROAD = 8;
function plan(sx, sy, gx, gy) {
  const [si, sj] = cellOf(sx, sy);
  let [gi, gj] = cellOf(gx, gy);
  if (cost[gj * GW + gi]) {
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
      if (closed[m] || occ[m]) continue;
      if (cost[m] && m !== goal && (!cost[s] || Math.hypot(ii - si, jj - sj) > INFLATE / RES + 1)) continue;
      if (di && dj && occ[j * GW + ii] && occ[jj * GW + i]) continue;
      const ng = g[n] + c * (road[m] ? 1 : OFFROAD);
      if (ng < g[m]) { g[m] = ng; came[m] = n; push(m, ng + h(ii, jj)); }
    }
  }
  if (came[goal] === -1 && goal !== s) return null;
  const cells = []; for (let n = goal; n !== -1; n = came[n]) cells.push(n);
  cells.reverse();
  const pts = cells.map(n => ({ x: (n % GW + 0.5) * RES, y: (((n / GW) | 0) + 0.5) * RES }));
  pts[0] = { x: sx, y: sy };
  const [ogi, ogj] = cellOf(gx, gy);
  if (gi === ogi && gj === ogj) pts[pts.length - 1] = { x: gx, y: gy };
  return smooth(pts);
}
function lineFree(a, b) {   // straight shortcut allowed only if it stays clear AND on walkways
  const d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.ceil(d / (RES / 3));
  for (let k = 1; k < n; k++) {
    const t = k / n, x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t;
    if (isCost(x, y) || !isRoad(x, y)) return false;
  }
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
function roadAt(x, y) {
  if (inRect(x, y, PLAZA.rect)) return PLAZA;
  let best = null, bd = 0.09;
  for (const r of ROADS) { const d = segDist(x, y, r.a, r.b); if (d < bd) { bd = d; best = r; } }
  return best;
}

/* ==========================================================================
   State
   ========================================================================== */
const S = {
  lang: 'en',
  mode: 'AUTO',
  estop: false,
  pose: { x: DOCK.x, y: DOCK.y, th: DOCK.th },
  v: 0, w: 0,
  nav: null,                // { key, goal:[x,y,th], path, wp, total, phase }
  trail: [],
  layers: { scan: false },
  scan: [], scanMin: null,
  battDrive: 85, battAux: 92,
  held: null,
  t0: performance.now(),
  lastDet: null,
  history: [],
  persona: { name_en: 'Gil', name_ko: '길이' },
  offer: null,
  place: null,              // building shown in the place card
  voice: 'idle',            // idle | listening | thinking | speaking
};

/* ==========================================================================
   i18n
   ========================================================================== */
const T = {
  en: {
    console: 'Console', map: 'Campus map', camera: 'Camera · YOLOv8', request: 'Ask Gil', tft: 'Onboard TFT',
    drive: 'Manual drive', send: 'SEND', drive_hint: 'Hold to drive. Cancels the active goal.',
    placeholder: 'Or type a question…',
    st_idle: 'Tap to talk', st_listening: 'Listening…', st_thinking: () => `${S.persona.name_en} is thinking…`, st_speaking: () => `${S.persona.name_en} is speaking`,
    st_blocked: 'Microphone blocked. Type your question instead.',
    tft_ready: () => [`Hi, I'm ${S.persona.name_en}!`, 'Ask me anything about campus'], tft_go: n => [n, 'Follow me'], tft_arrived: n => [n, 'We have arrived'],
    tft_manual: ['Manual drive', 'Operator in control'], tft_stop: ['STOP', 'Emergency stop active'],
    greet: () => `Hi, I'm ${S.persona.name_en}, your campus guide! Ask me where a building is, or anything about the campus.`,
    say_go: n => `Follow me, I'll take you to the ${n}!`, say_arrive: n => `We have arrived at the ${n}.`,
    say_unknown: "Sorry, I'm not sure. Try asking about a building, like the library, the cafe or the business school.",
    take_me: n => `Take me to the ${n}`,
    say_estop: 'Emergency stop activated.', say_blocked: 'Emergency stop is active. Release it first.',
    say_lang: 'Language set to English.', say_noplan: 'I could not find a path there.',
    chips: ['Where is the Business School?', 'When was the library built?', 'Where can I get Wi-Fi?', 'Who are you?'],
    searching: 'searching…', idle: 'idle',
    dir: 'Directions', ask: 'Ask Gil', exit: 'Exit', arrived: n => `Arrived at ${n}`,
    head: r => `Head along ${r}`, turn_l: r => `Turn left onto ${r}`, turn_r: r => `Turn right onto ${r}`, cont: r => `Continue on ${r}`, arrive_at: n => `Arrive at ${n}`,
    sheet: (s, m) => [`${s} s`, `${m} m`],
  },
  ko: {
    console: '콘솔', map: '캠퍼스 지도', camera: '카메라 · YOLOv8', request: '길이에게 질문', tft: '로봇 화면',
    drive: '수동 조종', send: '전송', drive_hint: '누르고 있으면 이동합니다. 현재 목표는 취소됩니다.',
    placeholder: '또는 질문을 입력하세요…',
    st_idle: '눌러서 말하기', st_listening: '듣고 있어요…', st_thinking: () => `${S.persona.name_ko}가 생각 중…`, st_speaking: () => `${S.persona.name_ko}가 말하는 중`,
    st_blocked: '마이크가 차단되었어요. 질문을 입력해 주세요.',
    tft_ready: () => [`안녕하세요, ${S.persona.name_ko}예요!`, '캠퍼스에 대해 물어보세요'], tft_go: n => [n, '저를 따라오세요'], tft_arrived: n => [n, '도착했습니다'],
    tft_manual: ['수동 조종', '운영자 제어 중'], tft_stop: ['정지', '비상 정지 작동 중'],
    greet: () => `안녕하세요, 캠퍼스 안내 로봇 ${S.persona.name_ko}예요! 건물 위치나 캠퍼스에 대해 무엇이든 물어보세요.`,
    say_go: n => `${n}(으)로 안내할게요. 저를 따라오세요!`, say_arrive: n => `${n}에 도착했습니다.`,
    say_unknown: '잘 모르겠어요. 도서관, 카페, 경영대학처럼 건물 이름으로 물어봐 주세요.',
    take_me: n => `${n}(으)로 안내`,
    say_estop: '비상 정지가 작동되었습니다.', say_blocked: '비상 정지 상태입니다. 먼저 해제해 주세요.',
    say_lang: '한국어로 설정되었습니다.', say_noplan: '그곳으로 가는 경로를 찾지 못했습니다.',
    chips: ['경영대학 어디에 있나요?', '도서관은 몇 시까지 해요?', '셔틀버스 어디서 타요?', '넌 누구야?'],
    searching: '탐색 중…', idle: '대기',
    dir: '길찾기', ask: '길이에게 질문', exit: '종료', arrived: n => `${n} 도착`,
    head: r => `${r} 따라 이동`, turn_l: r => `${r}에서 좌회전`, turn_r: r => `${r}에서 우회전`, cont: r => `${r} 직진`, arrive_at: n => `${n} 도착`,
    sheet: (s, m) => [`${s}초`, `${m} m`],
  },
};
const t = k => T[S.lang][k];
const tv = k => { const v = T[S.lang][k]; return typeof v === 'function' ? v() : v; };
const bname = b => (S.lang === 'ko' ? b.ko : b.en);
const rname = r => (S.lang === 'ko' ? r.ko : r.en);

function setLang(lang, silent) {
  S.lang = lang;
  document.documentElement.lang = lang;
  document.getElementById('lang-en').classList.toggle('on', lang === 'en');
  document.getElementById('lang-ko').classList.toggle('on', lang === 'ko');
  document.querySelectorAll('[data-i18n]').forEach(el => { const v = T[lang][el.dataset.i18n]; if (typeof v === 'string') el.textContent = v; });
  document.getElementById('ask-text').placeholder = t('placeholder');
  renderChips(); renderMapChips(); refreshTFT(); setOffer(S.offer); setVoice(S.voice); renderPlace();
  if (!silent) speak(t('say_lang'));
}

/* Activity goes to the browser console (no log panel in the demo UI). */
function log(src, msg) { console.debug(`[${src}] ${msg}`); }

/* ==========================================================================
   Chips
   ========================================================================== */
function svgIcon(path, cls = '') { return `<svg class="${cls}" viewBox="0 0 24 24" aria-hidden="true"><path d="${path}"/></svg>`; }
function renderChips() {
  const el = document.getElementById('chips');
  el.innerHTML = '';
  t('chips').forEach(q => {
    const b = document.createElement('button');
    b.textContent = q; b.onclick = () => handleText(q);
    el.appendChild(b);
  });
}
function renderMapChips() {
  const el = document.getElementById('map-chips');
  el.innerHTML = '';
  for (const k in B) {
    const btn = document.createElement('button');
    btn.className = 'mchip';
    btn.innerHTML = `<span class="mchip-ic" style="background:${B[k].color}">${svgIcon(ICON[k])}</span><span></span>`;
    btn.lastChild.textContent = bname(B[k]);
    btn.onclick = () => openPlace(k);
    el.appendChild(btn);
  }
}

/* ==========================================================================
   Navigation
   ========================================================================== */
function goTo(key) {
  if (S.estop) { speak(t('say_blocked')); return false; }
  if (S.mode === 'MANUAL') { document.getElementById('manual').checked = false; setManual(false); }
  const b = B[key];
  const path = plan(S.pose.x, S.pose.y, b.goal[0], b.goal[1]);
  if (!path) { speak(t('say_noplan')); setNavState('FAILED', 'err'); return false; }
  const end = path[path.length - 1];
  S.nav = { key, goal: [end.x, end.y, b.goal[2]], path, wp: 1, total: pathLen(path), phase: 'DRIVE' };
  ROBOT.sendGoal(key, end.x, end.y, b.goal[2]);
  setNavState('NAVIGATING', 'active');
  closePlace(); setOffer(null);
  if (view.z > 1.01) view.follow = true;
  refreshTFT(); renderNavUI();
  return true;
}
function cancelNav() {
  if (!S.nav) return;
  S.nav = null; S.v = 0; S.w = 0;
  ROBOT.cancelGoal();
  setNavState('IDLE'); refreshTFT(); renderNavUI();
}
function setNavState(txt, cls) {
  const el = document.getElementById('nav-state');
  el.textContent = txt; el.className = cls || '';
}

/* ---------- place card (tap a building or a chip) ---------- */
function openPlace(key) { S.place = key; renderPlace(); }
function closePlace() { S.place = null; renderPlace(); }
function renderPlace() {
  const card = document.getElementById('place');
  if (!S.place || S.nav) { card.hidden = true; return; }
  const b = B[S.place], kb = kbBuilding(S.place);
  card.hidden = false;
  card.querySelector('.pc-ic').style.background = b.color;
  card.querySelector('.pc-ic').innerHTML = svgIcon(ICON[S.place]);
  card.querySelector('.pc-name').textContent = bname(b);
  card.querySelector('.pc-cat').textContent = (S.lang === 'ko' ? b.cat_ko : b.cat_en) + (kb && kb.built ? ` · ${S.lang === 'ko' ? kb.built + '년' : 'Built ' + kb.built}` : '');
  card.querySelector('.pc-about').textContent = kb ? (kb[`about_${S.lang}`] || '') : '';
  card.querySelector('.pc-hours').textContent = kb && kb[`hours_${S.lang}`] ? kb[`hours_${S.lang}`] : '';
  card.querySelector('.pc-dir span').textContent = t('dir');
  card.querySelector('.pc-ask span').textContent = t('ask');
}

/* ==========================================================================
   Knowledge base + offline answers (mirrors server/app.py local_answer)
   ========================================================================== */
let KB = null;
async function loadKB() {
  try {
    const r = await fetch(CONFIG.knowledgeUrl, { cache: 'no-store' });
    if (!r.ok) return;
    KB = await r.json();
    S.persona = { name_en: KB.persona.name_en, name_ko: KB.persona.name_ko };
    refreshTFT(); setVoice(S.voice); renderPlace();
    if (!S.history.length) setGil(tv('greet'));
  } catch (e) { /* opened as a file: directions only */ }
}
const kbBuilding = id => (KB ? KB.buildings.find(b => b.id === id) : null);
const has = (q, words) => words.some(w => q.includes(w));
const W_GO = ['where', 'take me', 'go to', 'get to', 'way to', 'guide me', 'bring me', 'lead me', 'find the', '어디', '데려', '가고', '가는 길', '안내해', '가려면', '찾아'];
const W_HIST = ['history', 'built', 'old', 'founded', 'when', 'story', '역사', '언제', '지어', '오래', '세워'];
const W_HOURS = ['open', 'hour', 'close', 'time', '운영', '몇 시', '열어', '닫', '시간'];
const W_WHAT = ['what', 'inside', 'tell me', 'about', 'facilit', '무엇', '뭐', '뭐가', '소개', '알려'];
const W_HELLO = ['hello', 'hi ', 'hey', 'who are you', 'your name', '안녕', '누구', '이름'];

function localAnswer(text, lang) {
  const q = ` ${text.toLowerCase()} `, L = lang === 'ko' ? 'ko' : 'en';
  const buildings = KB ? KB.buildings : Object.keys(B).map(id => ({ id, aliases: B[id].keys, name_en: B[id].en, name_ko: B[id].ko }));
  const aliases = buildings.flatMap(b => b.aliases.map(a => [a.toLowerCase(), b])).sort((x, y) => y[0].length - x[0].length);
  const find = s => (aliases.find(([a]) => s.includes(a)) || [])[1] || null;
  let b = find(q);
  if (!b && has(q, [...W_HIST, ...W_HOURS, ...W_WHAT, ' it ', '그 건물', '거기'])) {
    for (const h of [...S.history].reverse()) { b = find(String(h.text).toLowerCase()); if (b) break; }
  }
  const out = (intent, target, response) => ({ intent, target, lang: L, response });
  if (b) {
    const name = b[`name_${L}`];
    if (has(q, W_HIST) && b[`history_${L}`]) return out('answer', b.id, b[`history_${L}`]);
    if (has(q, W_HOURS) && b[`hours_${L}`]) return out('answer', b.id, b[`hours_${L}`]);
    if (has(q, W_GO) || !KB) return out('navigate', b.id, T[L].say_go(name));
    return out('answer', b.id, b[`about_${L}`] || name);
  }
  if (KB) {
    for (const f of KB.faqs || []) if (has(q, f.keywords.map(k => k.toLowerCase()))) return out('answer', 'none', f[`a_${L}`]);
    if (has(q, W_HELLO)) return out('answer', 'none', KB.persona[`greeting_${L}`]);
    if (has(q, ['university', 'campus', 'school', '대학', '캠퍼스', '학교'])) return out('answer', 'none', KB.campus[`overview_${L}`]);
  }
  return out('clarify', 'none', T[L].say_unknown);
}

async function detectServer() {
  if (!location.protocol.startsWith('http')) return;
  try {
    const r = await fetch('/api/health', { cache: 'no-store' });
    if (!r.ok) return;
    const h = await r.json();
    CONFIG.intentEndpoint = '/api/ask';
    if (h.persona) S.persona = { name_en: h.persona.name_en, name_ko: h.persona.name_ko };
    document.getElementById('link-text').textContent = h.llm ? `AI · ${h.llm}` : 'AI · KNOWLEDGE BASE';
    refreshTFT(); setVoice(S.voice);
  } catch (e) { /* static hosting: answer from data/campus.json in the browser */ }
}

/* ==========================================================================
   Conversation
   ========================================================================== */
let busy = false;
async function handleText(text) {
  text = text.trim(); if (!text || busy) return;
  busy = true;
  const isKo = /[가-힣]/.test(text);
  if (isKo !== (S.lang === 'ko')) setLang(isKo ? 'ko' : 'en', true);
  setUser(text);
  setVoice('thinking');
  const t1 = performance.now();
  let res = null, source = 'browser';
  if (CONFIG.intentEndpoint) {
    try {
      const r = await fetch(CONFIG.intentEndpoint, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ text, lang: S.lang, history: S.history }), signal: AbortSignal.timeout(25000) });
      if (!r.ok) throw new Error(r.status);
      res = await r.json(); source = res.source || 'api';
    } catch (e) { log('llm', `server unreachable (${e.message}), answering in browser`); }
  }
  if (!res) res = localAnswer(text, S.lang);
  const wait = 650 - (performance.now() - t1);          // let the "thinking" state register
  if (wait > 0) await new Promise(r => setTimeout(r, wait));
  const key = B[res.target] ? res.target : null;
  document.getElementById('parse-ms').textContent = `${Math.round(performance.now() - t1)} ms · ${source}`;
  showJSON({ intent: res.intent, target: res.target, response: res.response });
  S.history.push({ role: 'user', text }, { role: 'guide', text: res.response });
  S.history = S.history.slice(-8);
  busy = false;
  speak(res.response);
  if (res.intent === 'navigate' && key) goTo(key);
  else setOffer(key);
}
function setUser(text) {
  const el = document.getElementById('b-user');
  el.hidden = !text; el.textContent = text || '';
}
let typeTimer = null;
function setGil(text) {
  const el = document.getElementById('gil-text');
  clearInterval(typeTimer);
  let i = 0; el.textContent = '';
  typeTimer = setInterval(() => { i += 2; el.textContent = text.slice(0, i); if (i >= text.length) clearInterval(typeTimer); }, 18);
  document.getElementById('gil-name').textContent = S.lang === 'ko' ? S.persona.name_ko : S.persona.name_en;
}
function setOffer(key) {
  S.offer = key;
  const btn = document.getElementById('offer');
  btn.hidden = !key;
  if (key) btn.querySelector('span').textContent = t('take_me')(bname(B[key]));
}
function showJSON(o) {
  const esc = s => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
  const val = v => v === null || v === undefined ? '<span class="jn">null</span>' : `<span class="js">"${esc(v)}"</span>`;
  document.getElementById('json').innerHTML = '{\n' + Object.keys(o).map(k => `  <span class="jk">"${k}"</span>: ${val(o[k])}`).join(',\n') + '\n}';
}
function submitText() {
  const el = document.getElementById('ask-text');
  handleText(el.value); el.value = '';
}

/* ---------- voice state + animation ---------- */
function setVoice(state) {
  S.voice = state;
  document.getElementById('orb-wrap').dataset.state = state;
  const key = { idle: 'st_idle', listening: 'st_listening', thinking: 'st_thinking', speaking: 'st_speaking' }[state];
  document.getElementById('orb-status').textContent = tv(key);
  document.getElementById('speaker').textContent = state === 'speaking' ? 'SPK ▶ PLAYING' : 'SPK IDLE';
}
const bars = [...document.querySelectorAll('#wave i')];
let level = null;   // live microphone level while listening, when the browser allows it
// Phones allow only one user of the microphone: opening a second stream for the meter
// stops speech recognition from getting the mic on the next turn. Desktop only.
const SINGLE_MIC = matchMedia('(pointer: coarse)').matches || /Android|iPhone|iPad|iPod/i.test(navigator.userAgent);
async function startLevel() {
  if (SINGLE_MIC || level) return;
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const an = ctx.createAnalyser(); an.fftSize = 64;
    ctx.createMediaStreamSource(stream).connect(an);
    level = { stream, ctx, an, buf: new Uint8Array(an.frequencyBinCount) };
    document.getElementById('wave').classList.add('live');
  } catch (e) { level = null; }
}
function stopLevel() {
  if (!level) return;
  level.stream.getTracks().forEach(tr => tr.stop()); level.ctx.close();
  level = null;
  document.getElementById('wave').classList.remove('live');
  bars.forEach(b => (b.style.height = ''));
}
function drawLevel() {
  if (!level) return;
  level.an.getByteFrequencyData(level.buf);
  bars.forEach((b, i) => {
    const v = level.buf[2 + ((i * 1.6) | 0)] / 255;
    b.style.height = `${6 + v * 34}px`;
  });
}

/* ---------- speech in ---------- */
let rec = null;
const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
const sleep = ms => new Promise(r => setTimeout(r, ms));
function makeRecognizer() {
  const r = new SR();
  r.continuous = false; r.interimResults = true; r.maxAlternatives = 1;
  r.lang = S.lang === 'ko' ? 'ko-KR' : 'en-US';
  r.st = { started: false, error: null, finalText: '', ended: false };
  const current = () => rec === r;
  r.onstart = () => {
    r.st.started = true;
    if (!current()) return;
    document.getElementById('orb-status').textContent = tv('st_listening');
    startLevel();
  };
  r.onresult = e => {
    if (!current()) return;
    let interim = '';
    for (let i = e.resultIndex; i < e.results.length; i++) {
      if (e.results[i].isFinal) r.st.finalText += e.results[i][0].transcript; else interim += e.results[i][0].transcript;
    }
    setUser(r.st.finalText + interim);
  };
  r.onerror = e => { r.st.error = e.error; };
  r.onend = () => {
    r.st.ended = true;
    if (!current()) return;
    rec = null;
    stopLevel();
    const text = r.st.finalText.trim();
    if (text) { handleText(text); return; }
    setVoice('idle');
    if (r.st.error && r.st.error !== 'aborted') micMessage(r.st.error);
    else if (r.st.started) micMessage('no-speech');
  };
  return r;
}
function micMessage(code) {
  const ko = S.lang === 'ko';
  const msg = {
    'not-allowed': ko ? '마이크 권한이 거부되었어요. 주소창의 마이크 아이콘에서 허용해 주세요.' : 'Microphone permission was denied. Allow it from the mic icon in the address bar.',
    'service-not-allowed': ko ? '이 브라우저에서는 음성 인식을 쓸 수 없어요. Chrome이나 Edge를 사용해 주세요.' : 'Speech recognition is blocked here. Use Chrome or Edge over http://localhost or https.',
    'audio-capture': ko ? '마이크를 찾을 수 없어요. 마이크 연결을 확인해 주세요.' : 'No microphone found. Check that one is connected.',
    'network': ko ? '음성 인식 서버에 연결할 수 없어요. 인터넷 연결을 확인해 주세요.' : "Can't reach the speech service. Check the internet connection.",
    'no-speech': ko ? '말소리가 들리지 않았어요. 다시 눌러서 말해 주세요.' : "I didn't hear anything. Tap and try again.",
    'unsupported': ko ? '이 브라우저는 음성 입력을 지원하지 않아요. Chrome이나 Edge를 사용하거나 아래에 입력해 주세요.' : "This browser doesn't support voice input. Use Chrome or Edge, or type below.",
    'insecure': ko ? '음성 입력은 http://localhost 또는 https 주소에서만 작동해요.' : 'Voice input only works on http://localhost or an https address, not a local file.',
  }[code] || (ko ? '음성 입력을 시작할 수 없어요. 아래에 입력해 주세요.' : "Couldn't start voice input. Type your question below.");
  const el = document.getElementById('orb-status');
  el.textContent = msg; el.classList.add('warn');
  clearTimeout(micMessage._t);
  micMessage._t = setTimeout(() => { el.classList.remove('warn'); if (S.voice === 'idle') el.textContent = tv('st_idle'); }, 6000);
}
let micStarting = false;
function toggleMic() {
  if (micStarting) return;
  if (busy) {
    const el = document.getElementById('orb-status');
    el.textContent = S.lang === 'ko' ? '답변을 준비하고 있어요. 잠시만요…' : 'Still working on the answer, one moment…';
    return;
  }
  if (S.voice === 'listening') {                      // second tap while listening = done talking
    if (rec) { try { rec.stop(); } catch (e) {} } else setVoice('idle');
    return;
  }
  if (!SR) { micMessage('unsupported'); document.getElementById('ask-text').focus(); return; }
  if (!window.isSecureContext) { micMessage('insecure'); return; }
  startListening();
}
async function startListening() {
  micStarting = true;
  const status = document.getElementById('orb-status');
  status.classList.remove('warn'); clearTimeout(micMessage._t);
  setVoice('listening'); setUser('');
  status.textContent = S.lang === 'ko' ? '마이크를 기다리는 중… (권한을 허용해 주세요)' : 'Waiting for the microphone… (allow access if asked)';

  // Gil may still be talking: phones can't record while the speaker is in use.
  if ('speechSynthesis' in window && (speechSynthesis.speaking || speechSynthesis.pending)) {
    speechSynthesis.cancel(); await sleep(350);
  }
  // Release the previous session and give the phone a moment to free the mic.
  const old = rec; rec = null;
  if (old && !old.st.ended) {
    try { old.abort(); } catch (e) {}
    for (let i = 0; i < 10 && !old.st.ended; i++) await sleep(60);
  }
  stopLevel();

  let r = makeRecognizer(); rec = r;
  try { r.start(); }
  catch (e) {                                         // InvalidStateError: the old session hadn't let go yet
    await sleep(400);
    r = makeRecognizer(); rec = r;
    try { r.start(); } catch (e2) { rec = null; micStarting = false; setVoice('idle'); micMessage(); return; }
  }
  micStarting = false;
  setTimeout(() => {
    if (rec === r && !r.st.started && S.voice === 'listening') { try { r.abort(); } catch (e) {} rec = null; setVoice('idle'); micMessage(); }
  }, 15000);
}

/* ---------- speech out ---------- */
function pickVoice(lang) {
  if (!('speechSynthesis' in window)) return null;
  const vs = speechSynthesis.getVoices().filter(v => v.lang && v.lang.toLowerCase().startsWith(lang));
  return vs.find(v => /google|natural|neural|online/i.test(v.name)) || vs[0] || null;
}
function speak(text) {
  setGil(text);
  log('tts', text);
  ROBOT.say(text, S.lang);
  if (S.voice === 'listening') return;              // don't talk over the visitor (or into the mic)
  setVoice('speaking');
  let finished = false;
  const done = () => { if (finished) return; finished = true; if (S.voice === 'speaking') setVoice('idle'); };
  const estimate = Math.min(12000, 1400 + text.length * 55);
  if ('speechSynthesis' in window) {
    speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    u.lang = S.lang === 'ko' ? 'ko-KR' : 'en-US';
    const v = pickVoice(S.lang); if (v) u.voice = v;
    let started = false;
    u.onstart = () => { started = true; };
    u.onend = done; u.onerror = done;
    speechSynthesis.speak(u);
    setTimeout(() => { if (!started) setTimeout(done, estimate - 1200); }, 1200);   // no audio output: fall back to an estimate
    setTimeout(done, estimate + 4000);                // mobile browsers sometimes never fire onend
  } else setTimeout(done, estimate);
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
    S.v = 0; S.w = 0;
    if (S.nav) { S.nav = null; renderNavUI(); }
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
  let lines = t('tft_ready')(), state = 'READY', alert = false;
  if (S.estop) { lines = t('tft_stop'); state = 'E-STOP'; alert = true; }
  else if (S.mode === 'MANUAL') { lines = t('tft_manual'); state = 'MANUAL'; }
  else if (S.nav) {
    const n = bname(B[S.nav.key]);
    lines = S.nav.phase === 'DONE' ? t('tft_arrived')(n) : t('tft_go')(n);
    state = S.nav.phase === 'DONE' ? 'ARRIVED' : 'MOVING';
  }
  document.getElementById('tft-main').textContent = lines[0];
  document.getElementById('tft-sub').textContent = lines[1];
  document.getElementById('tft-state').textContent = state;
  document.getElementById('tft-mode').textContent = S.estop ? 'STOP' : S.mode;
  document.querySelector('.tft').classList.toggle('alert', alert);
  ROBOT.tft({ main: lines[0], sub: lines[1], state, mode: S.estop ? 'STOP' : S.mode, batt: Math.round(S.battDrive), lang: S.lang });
}

/* ==========================================================================
   Simulation step
   ========================================================================== */
function step(dt) {
  if (ROBOT.live()) { ROBOT.step(dt); return; }
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
  const b = B[N.key], kb = kbBuilding(N.key);
  log('yolo', `landmark lock ${b.slug}: h/H ≥ 0.62 → arrived`);
  const about = kb ? (kb[`fun_fact_${S.lang}`] || kb[`about_${S.lang}`] || '') : '';
  speak(`${t('say_arrive')(bname(b))} ${about}`.trim());
  refreshTFT(); updateNavUI();
  setTimeout(() => { if (S.nav === N) { S.nav = null; setNavState('IDLE'); refreshTFT(); renderNavUI(); } }, 5000);
}

/* ---------- LiDAR (LDS: 360 samples, 0.12–3.5 m; we sample every 2°) ---------- */
function scan() {
  if (ROBOT.live()) return;                         // real /scan arrives through the bridge
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
   Map: street-map style rendering, zoom / pan, place card, directions
   ========================================================================== */
const mapCanvas = document.getElementById('map');
const mctx = mapCanvas.getContext('2d');
const view = { z: 1, cx: WORLD_W / 2, cy: WORLD_H / 2, follow: false };
const MAPC = {
  outside: '#E8EAED', land: '#F4F3F0', lawn: '#C9E9D0', lawnEdge: '#B9DFC2',
  water: '#A8D8F7', waterText: '#3678C1', plaza: '#E9E7E2', plazaLine: '#DEDBD4',
  road: '#FFFFFF', roadEdge: '#D6D8DC', bldTop: '#E6E5E2', bldSide: '#D3D1CC', bldEdge: '#CAC8C2',
  label: '#3C4043', roadLabel: '#70757A', route: '#4285F4', routeEdge: '#1967D2', me: '#1A73E8', pin: '#EA4335',
};
const iconPaths = {}; for (const k in ICON) iconPaths[k] = new Path2D(ICON[k]);
let K = 1;                                            // device px per CSS px

function sizeMap() {
  const r = mapCanvas.getBoundingClientRect(), dpr = Math.min(2, window.devicePixelRatio || 1);
  mapCanvas.width = Math.round(r.width * dpr); mapCanvas.height = Math.round(r.height * dpr);
  K = mapCanvas.width / Math.max(1, r.width);
}
const scale = () => Math.min(mapCanvas.width / WORLD_W, mapCanvas.height / WORLD_H) * view.z;   // fit, then zoom
function clampView() {
  const sc = scale(), hw = mapCanvas.width / 2 / sc, hh = mapCanvas.height / 2 / sc;
  view.cx = hw * 2 >= WORLD_W ? WORLD_W / 2 : Math.min(WORLD_W - hw, Math.max(hw, view.cx));
  view.cy = hh * 2 >= WORLD_H ? WORLD_H / 2 : Math.min(WORLD_H - hh, Math.max(hh, view.cy));
}
const sx = x => (x - view.cx) * scale() + mapCanvas.width / 2;
const sy = y => (y - view.cy) * scale() + mapCanvas.height / 2;
function toWorld(cssX, cssY) {
  const px = cssX * K, py = cssY * K, sc = scale();
  return { x: (px - mapCanvas.width / 2) / sc + view.cx, y: (py - mapCanvas.height / 2) / sc + view.cy };
}
function zoomBy(f, cssX, cssY) {
  const r = mapCanvas.getBoundingClientRect();
  if (cssX === undefined) { cssX = r.width / 2; cssY = r.height / 2; }
  const before = toWorld(cssX, cssY);
  view.z = Math.min(3.2, Math.max(1, view.z * f));
  const after = toWorld(cssX, cssY);
  view.cx += before.x - after.x; view.cy += before.y - after.y;
  clampView();
}
function recenter() {
  view.follow = true;
  if (view.z < 1.8) zoomBy(1.8 / view.z);
}
function rrect(g, x, y, w, h, r) { g.beginPath(); g.roundRect(x, y, w, h, r); }
function haloText(g, text, x, y, color, font) {
  g.font = font; g.lineJoin = 'round';
  g.strokeStyle = 'rgba(255,255,255,0.95)'; g.lineWidth = 3.5 * K; g.strokeText(text, x, y);
  g.fillStyle = color; g.fillText(text, x, y);
}

/* ---------- interaction ---------- */
let drag = null;
mapCanvas.addEventListener('pointerdown', e => {
  drag = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false };
  mapCanvas.setPointerCapture(e.pointerId);
});
mapCanvas.addEventListener('pointermove', e => {
  if (!drag) { mapCanvas.style.cursor = hitBuilding(e) ? 'pointer' : 'grab'; return; }
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.hypot(dx, dy) > 4) drag.moved = true;
  if (drag.moved) {
    const sc = scale() / K;
    view.cx = drag.cx - dx / sc; view.cy = drag.cy - dy / sc; view.follow = false; clampView();
    mapCanvas.style.cursor = 'grabbing';
  }
});
mapCanvas.addEventListener('pointerup', e => {
  if (drag && !drag.moved) { const k = hitBuilding(e); if (k) openPlace(k); else closePlace(); }
  drag = null;
});
mapCanvas.addEventListener('wheel', e => {
  e.preventDefault();
  const r = mapCanvas.getBoundingClientRect();
  zoomBy(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX - r.left, e.clientY - r.top);
  view.follow = false;
}, { passive: false });
function hitBuilding(e) {
  const r = mapCanvas.getBoundingClientRect(), p = toWorld(e.clientX - r.left, e.clientY - r.top);
  for (const k in B) { const q = B[k].rect; if (inRect(p.x, p.y, [q[0] - 0.04, q[1] - 0.04, q[2] + 0.04, q[3] + 0.04])) return k; }
  return null;
}

/* ---------- drawing ---------- */
function drawMap() {
  const g = mctx, W = mapCanvas.width, H = mapCanvas.height, sc = scale(), P = S.pose;
  if (view.follow && view.z > 1.01) {
    view.cx += (P.x - view.cx) * 0.08; view.cy += (P.y - view.cy) * 0.08; clampView();
  }
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = MAPC.outside; g.fillRect(0, 0, W, H);
  g.fillStyle = MAPC.land; g.fillRect(sx(0), sy(0), WORLD_W * sc, WORLD_H * sc);

  // lawns: each city block, inset from the walkways
  for (let bi = 0; bi < 3; bi++) for (let bj = 0; bj < 3; bj++) {
    const pad = 0.1, x0 = XS[bi] + (bi ? pad : 0), x1 = XS[bi + 1] - (bi < 2 ? pad : 0), y0 = YS[bj] + (bj ? pad : 0), y1 = YS[bj + 1] - (bj < 2 ? pad : 0);
    rrect(g, sx(x0), sy(y0), (x1 - x0) * sc, (y1 - y0) * sc, 0.05 * sc);
    g.fillStyle = MAPC.lawn; g.fill();
  }
  // plaza
  const pr = PLAZA.rect;
  rrect(g, sx(pr[0]), sy(pr[1]), (pr[2] - pr[0]) * sc, (pr[3] - pr[1]) * sc, 0.06 * sc);
  g.fillStyle = MAPC.plaza; g.fill();
  g.save(); g.clip();
  g.strokeStyle = MAPC.plazaLine; g.lineWidth = 1 * K;
  for (let x = pr[0]; x < pr[2]; x += 0.1) { g.beginPath(); g.moveTo(sx(x), sy(pr[1])); g.lineTo(sx(x), sy(pr[3])); g.stroke(); }
  for (let y = pr[1]; y < pr[3]; y += 0.1) { g.beginPath(); g.moveTo(sx(pr[0]), sy(y)); g.lineTo(sx(pr[2]), sy(y)); g.stroke(); }
  g.restore();
  // fountain
  const fx = sx(FOUNTAIN.x), fy = sy(FOUNTAIN.y), fr = FOUNTAIN.r * sc;
  g.beginPath();
  for (let i = 0; i < 8; i++) { const a = Math.PI / 8 + i * Math.PI / 4; g.lineTo(fx + Math.cos(a) * fr * 1.12, fy + Math.sin(a) * fr * 1.12); }
  g.closePath(); g.fillStyle = '#D9D5CC'; g.fill();
  g.beginPath();
  for (let i = 0; i < 8; i++) { const a = Math.PI / 8 + i * Math.PI / 4; g.lineTo(fx + Math.cos(a) * fr * 0.88, fy + Math.sin(a) * fr * 0.88); }
  g.closePath(); g.fillStyle = MAPC.water; g.fill();
  g.beginPath(); g.arc(fx, fy, fr * 0.22, 0, Math.PI * 2); g.fillStyle = '#D9D5CC'; g.fill();
  g.textAlign = 'center'; g.textBaseline = 'middle';
  haloText(g, S.lang === 'ko' ? '분수대' : 'Fountain', fx, fy + fr + 9 * K, MAPC.waterText, `italic 500 ${10 * K}px Roboto, sans-serif`);
  // lake
  g.beginPath(); g.ellipse(sx(LAKE.x), sy(LAKE.y), LAKE.rx * sc, LAKE.ry * sc, 0, 0, Math.PI * 2);
  g.fillStyle = MAPC.water; g.fill();
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.font = `italic 500 ${11.5 * K}px Roboto, sans-serif`; g.fillStyle = MAPC.waterText;
  g.fillText(S.lang === 'ko' ? LAKE.ko : LAKE.en, sx(LAKE.x), sy(LAKE.y));

  // walkways: casing then fill
  g.lineCap = 'round';
  for (const pass of [0, 1]) for (const r of ROADS) {
    g.beginPath(); g.moveTo(sx(r.a[0]), sy(r.a[1])); g.lineTo(sx(r.b[0]), sy(r.b[1]));
    g.strokeStyle = pass ? MAPC.road : MAPC.roadEdge;
    g.lineWidth = r.w * sc * (r.minor ? 0.8 : 0.85) + (pass ? 0 : 2 * K);
    g.stroke();
  }
  // dock pad
  const dk = 0.09;
  rrect(g, sx(DOCK.x - dk), sy(DOCK.y - dk * 0.6), dk * 2 * sc, dk * 1.4 * sc, 0.03 * sc);
  g.fillStyle = '#D2E3FC'; g.fill(); g.strokeStyle = '#AECBFA'; g.lineWidth = 1 * K; g.stroke();
  g.font = `500 ${9.5 * K}px Roboto, sans-serif`; g.fillStyle = '#1967D2';
  g.fillText('DOCK', sx(DOCK.x), sy(DOCK.y + dk * 1.25));

  // buildings with a hint of extrusion
  for (const k in B) {
    const q = B[k].rect, ext = Math.max(2, 0.018 * sc);
    rrect(g, sx(q[0]), sy(q[1]) + ext, (q[2] - q[0]) * sc, (q[3] - q[1]) * sc, 0.015 * sc); g.fillStyle = MAPC.bldSide; g.fill();
    rrect(g, sx(q[0]), sy(q[1]), (q[2] - q[0]) * sc, (q[3] - q[1]) * sc, 0.015 * sc); g.fillStyle = MAPC.bldTop; g.fill();
    g.strokeStyle = MAPC.bldEdge; g.lineWidth = 1 * K; g.stroke();
  }

  // route
  const N = S.nav;
  if (N && N.phase !== 'DONE') {
    const pts = [{ x: P.x, y: P.y }, ...N.path.slice(N.wp)];
    for (const pass of [0, 1]) {
      g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(sx(p.x), sy(p.y)) : g.moveTo(sx(p.x), sy(p.y))));
      g.strokeStyle = pass ? MAPC.route : MAPC.routeEdge; g.lineJoin = 'round';
      g.lineWidth = (pass ? 6 : 9) * K; g.stroke();
    }
  }
  // dotted walk from the stop point to the building
  if (N) {
    const q = B[N.key].rect, cx = (q[0] + q[2]) / 2, cy = (q[1] + q[3]) / 2;
    const [gx, gy] = N.goal, d = Math.hypot(cx - gx, cy - gy), n = Math.max(2, Math.floor(d * sc / (8 * K)));
    g.fillStyle = MAPC.route;
    for (let i = 1; i < n; i++) { g.beginPath(); g.arc(sx(gx + (cx - gx) * i / n), sy(gy + (cy - gy) * i / n), 2.2 * K, 0, Math.PI * 2); g.fill(); }
  }

  // street names
  g.textAlign = 'center'; g.textBaseline = 'middle';
  for (const r of ROADS) {
    if (r.minor) continue;
    const vertical = r.a[0] === r.b[0];
    const lx = vertical ? r.a[0] : 3.70, ly = vertical ? (r.a[0] < 2 ? 0.55 : 2.55) : r.a[1];
    g.save(); g.translate(sx(lx), sy(ly)); if (vertical) g.rotate(-Math.PI / 2);
    haloText(g, rname(r), 0, 0, MAPC.roadLabel, `500 ${10.5 * K}px Roboto, sans-serif`);
    g.restore();
  }
  haloText(g, S.lang === 'ko' ? PLAZA.ko : PLAZA.en, sx((pr[0] + pr[2]) / 2), sy(pr[1] + 0.1), MAPC.roadLabel, `500 ${10.5 * K}px Roboto, sans-serif`);

  // sensors layer
  if (S.layers.scan) {
    const f = 31.1 * Math.PI / 180;
    g.fillStyle = 'rgba(66,133,244,0.08)';
    g.beginPath(); g.moveTo(sx(P.x), sy(P.y)); g.arc(sx(P.x), sy(P.y), 0.9 * sc, P.th - f, P.th + f); g.closePath(); g.fill();
    g.fillStyle = '#0F9D58';
    for (const s of S.scan) { const a = P.th + s.a; g.fillRect(sx(P.x + Math.cos(a) * s.r) - 1.5 * K, sy(P.y + Math.sin(a) * s.r) - 1.5 * K, 3 * K, 3 * K); }
  }

  // places: icon + label, or a red pin when selected
  const target = N ? N.key : S.place;
  for (const k in B) {
    const b = B[k], q = b.rect, cx = sx((q[0] + q[2]) / 2), cy = sy((q[1] + q[3]) / 2), sel = k === target;
    if (sel) {
      g.save(); g.translate(cx - 17 * K, cy - 36 * K); g.scale(1.45 * K, 1.45 * K);
      g.shadowColor = 'rgba(0,0,0,.3)'; g.shadowBlur = 4; g.shadowOffsetY = 1;
      g.fillStyle = MAPC.pin; g.fill(iconPaths.pin);
      g.shadowColor = 'transparent'; g.strokeStyle = '#B31412'; g.lineWidth = 0.6; g.stroke(iconPaths.pin);
      g.restore();
      g.beginPath(); g.arc(cx, cy - 23 * K, 3.6 * K, 0, Math.PI * 2); g.fillStyle = '#B31412'; g.fill();
    } else {
      g.beginPath(); g.arc(cx, cy - 6 * K, 11 * K, 0, Math.PI * 2);
      g.fillStyle = b.color; g.shadowColor = 'rgba(0,0,0,.25)'; g.shadowBlur = 3 * K; g.shadowOffsetY = 1 * K; g.fill();
      g.shadowColor = 'transparent'; g.strokeStyle = '#fff'; g.lineWidth = 1.5 * K; g.stroke();
      g.save(); g.translate(cx - 7 * K, cy - 13 * K); g.scale(14 * K / 24, 14 * K / 24); g.fillStyle = '#fff'; g.fill(iconPaths[k]); g.restore();
    }
    haloText(g, bname(b), cx, cy + (sel ? 4 : 14) * K, sel ? '#C5221F' : b.color, `${sel ? 700 : 600} ${11.5 * K}px Roboto, "IBM Plex Sans KR", sans-serif`);
  }

  // you-are-here: halo, heading beam, blue dot
  const rx = sx(P.x), ry = sy(P.y), col = S.estop ? '#D93025' : MAPC.me;
  g.beginPath(); g.arc(rx, ry, 22 * K, 0, Math.PI * 2); g.fillStyle = S.estop ? 'rgba(217,48,37,.15)' : 'rgba(26,115,232,.13)'; g.fill();
  const beam = g.createRadialGradient(rx, ry, 4 * K, rx, ry, 46 * K);
  beam.addColorStop(0, S.estop ? 'rgba(217,48,37,.45)' : 'rgba(26,115,232,.45)'); beam.addColorStop(1, 'rgba(26,115,232,0)');
  g.beginPath(); g.moveTo(rx, ry); g.arc(rx, ry, 46 * K, P.th - 0.55, P.th + 0.55); g.closePath(); g.fillStyle = beam; g.fill();
  g.beginPath(); g.arc(rx, ry, 10 * K, 0, Math.PI * 2); g.fillStyle = '#fff';
  g.shadowColor = 'rgba(0,0,0,.35)'; g.shadowBlur = 4 * K; g.fill(); g.shadowColor = 'transparent';
  g.beginPath(); g.arc(rx, ry, 7.5 * K, 0, Math.PI * 2); g.fillStyle = col; g.fill();

  // scale bar
  const barM = view.z > 1.8 ? 0.25 : 0.5, bw = barM * sc, bx = 12 * K, by = H - 14 * K;
  g.fillStyle = 'rgba(255,255,255,.85)'; rrect(g, bx - 6 * K, by - 18 * K, bw + 12 * K, 24 * K, 3 * K); g.fill();
  g.strokeStyle = '#5F6368'; g.lineWidth = 1.5 * K;
  g.beginPath(); g.moveTo(bx, by - 4 * K); g.lineTo(bx, by); g.lineTo(bx + bw, by); g.lineTo(bx + bw, by - 4 * K); g.stroke();
  g.fillStyle = '#3C4043'; g.font = `500 ${10 * K}px Roboto, sans-serif`; g.textAlign = 'left'; g.textBaseline = 'alphabetic';
  g.fillText(`${barM * 100} cm`, bx + 2 * K, by - 7 * K);
}

/* ---------- directions banner + bottom sheet ---------- */
function renderNavUI() {
  const nav = !!S.nav;
  document.getElementById('map-chips').hidden = nav;
  document.getElementById('nav-banner').hidden = !nav;
  document.getElementById('nav-sheet').hidden = !nav;
  if (nav) renderPlace();
}
function updateNavUI() {
  const N = S.nav; if (!N) return;
  const P = S.pose, name = bname(B[N.key]);
  const banner = document.getElementById('nav-banner'), ic = document.getElementById('nb-icon');
  let text, icon = 'straight', dist = '';
  if (N.phase === 'DONE') { text = t('arrived')(name); icon = 'pin'; banner.classList.add('done'); }
  else {
    banner.classList.remove('done');
    const wp = Math.min(N.wp, N.path.length - 1), next = N.path[wp];
    const d = Math.hypot(next.x - P.x, next.y - P.y);
    if (wp === N.path.length - 1) { text = t('arrive_at')(name); icon = 'pin'; }
    else {
      const prev = wp > 0 ? N.path[wp - 1] : P, after = N.path[wp + 1];
      const ax = next.x - prev.x, ay = next.y - prev.y, bx = after.x - next.x, by = after.y - next.y;
      const ang = Math.atan2(ax * by - ay * bx, ax * bx + ay * by);
      const r = roadAt((next.x + after.x) / 2, (next.y + after.y) / 2), rn = r ? rname(r) : '';
      if (Math.abs(ang) < 0.4) { text = t('cont')(rn); icon = 'straight'; }
      else if (ang > 0) { text = t('turn_r')(rn); icon = 'right'; }     // y-down frame: positive cross = clockwise
      else { text = t('turn_l')(rn); icon = 'left'; }
    }
    dist = `${Math.round(d * 100)} cm`;
  }
  if (ic.dataset.icon !== icon) { ic.innerHTML = svgIcon(ICON[icon]); ic.dataset.icon = icon; }
  document.getElementById('nb-text').textContent = text;
  document.getElementById('nb-dist').textContent = dist;

  let rem = 0;
  if (N.phase === 'DRIVE') {
    const wp = Math.min(N.wp, N.path.length - 1);
    rem = Math.hypot(N.path[wp].x - P.x, N.path[wp].y - P.y);
    for (let i = wp; i < N.path.length - 1; i++) rem += Math.hypot(N.path[i + 1].x - N.path[i].x, N.path[i + 1].y - N.path[i].y);
  }
  const [s, m] = t('sheet')(Math.ceil(rem / VMAX / CONFIG.simSpeed), rem.toFixed(2));
  document.getElementById('ns-time').textContent = s;
  document.getElementById('ns-dist').textContent = `${m} · ${name}`;
  document.getElementById('ns-exit').textContent = t('exit');
}
/* ==========================================================================
   Camera rendering + landmark detection
   ========================================================================== */
const cam = document.getElementById('cam'), cctx = cam.getContext('2d');
const CW = 640, CH = 360, HFOV = 62.2 * Math.PI / 180, FX = (CW / 2) / Math.tan(HFOV / 2), HZ = CH * 0.46, CAM_H = 0.10;

function drawCam(now) {
  if (ROBOT.drawCamera(cctx, CW, CH)) return;       // live robot camera
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
   Loop / boot
   ========================================================================== */
function toggleLayer(k) {
  S.layers[k] = !S.layers[k];
  document.getElementById('t-' + k).classList.toggle('on', S.layers[k]);
}

let last = performance.now(), lastScan = 0, lastTel = 0, frames = 0, fpsT = performance.now();
function loop(now) {
  const dt = Math.min(0.05, (now - last) / 1000) * CONFIG.simSpeed; last = now;
  step(dt);
  if (now - lastScan > 100) { scan(); lastScan = now; }
  drawMap();
  drawCam(now);
  drawLevel();
  if (now - lastTel > 100) { telemetry(); updateNavUI(); lastTel = now; }
  frames++; if (now - fpsT > 1000) { document.getElementById('fps').textContent = frames; frames = 0; fpsT = now; }
  requestAnimationFrame(loop);
}

new ResizeObserver(() => { sizeMap(); clampView(); }).observe(mapCanvas.parentElement);
if ('speechSynthesis' in window) speechSynthesis.onvoiceschanged = () => {};   // warm up the voice list

(function boot() {
  setLang('en', true);
  sizeMap();
  scan();
  setGil(tv('greet'));
  renderNavUI();
  loadKB();
  detectServer();
  requestAnimationFrame(loop);
})();
