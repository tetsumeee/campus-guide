/* ==========================================================================
   CGR-1 robot bridge (rosbridge, plain WebSocket JSON — no libraries)
   Does nothing unless ROBOT_CONFIG.enabled is true (js/robot-config.js) or
   the page is opened with ?robot=on. While the robot is unreachable the
   dashboard keeps running its simulation and this keeps retrying.

   Dashboard hooks it fills in (see the ROBOT object in dashboard.js):
     live(), step(dt), sendGoal(), cancelGoal(), estop(), tft(), say(), drawCamera()
   ========================================================================== */
(function () {
  const C = window.ROBOT_CONFIG || {};
  const param = new URLSearchParams(location.search).get('robot');
  const enabled = param ? !/^(off|0|false)$/i.test(param) : !!C.enabled;
  if (!enabled) return;
  const URL_ = param && /^wss?:\/\//.test(param) ? param : C.url;
  const TP = C.topics || {};
  const MT = Object.assign({ x0: 0, y0: 0, yaw: 0, scale: 1 }, C.mapTransform || {});

  /* ---------- status chip in the top bar ---------- */
  const chip = document.createElement('span');
  chip.className = 'chip'; chip.id = 'robot-chip'; chip.title = URL_;
  chip.innerHTML = '<i class="led warn"></i><span>ROBOT · CONNECTING</span>';
  document.querySelector('.bar-status').prepend(chip);
  const setChip = (text, led) => { chip.lastChild.textContent = text; chip.firstChild.className = 'led ' + led; };

  /* ---------- minimal rosbridge client ---------- */
  const bridge = {
    ws: null, open: false, subs: {}, adv: {}, calls: 0,
    connect() {
      if (location.protocol === 'https:' && URL_.startsWith('ws:')) {
        setChip('ROBOT · NEEDS http:// PAGE', 'red');
        console.warn('[robot] an https page cannot open ws:// — serve the dashboard over http (python server/app.py) or use wss://');
        return;
      }
      let ws;
      try { ws = new WebSocket(URL_); } catch (e) { return this.retry(); }
      this.ws = ws;
      ws.onopen = () => {
        this.open = true; this.adv = {};
        for (const t in this.subs) this.send({ op: 'subscribe', topic: t, type: this.subs[t].type, throttle_rate: this.subs[t].throttle, queue_length: 1 });
        onConnect();
      };
      ws.onmessage = e => {
        let m; try { m = JSON.parse(e.data); } catch (_) { return; }
        if (m.op === 'publish' && this.subs[m.topic]) this.subs[m.topic].cb(m.msg);
      };
      ws.onclose = () => { const was = this.open; this.open = false; if (was) onDisconnect(); this.retry(); };
      ws.onerror = () => {};
    },
    retry() { setTimeout(() => this.connect(), 3000); },
    send(o) { if (this.open) this.ws.send(JSON.stringify(o)); },
    subscribe(topic, type, cb, throttle = 0) {
      if (!topic) return;
      this.subs[topic] = { type, cb, throttle };
      this.send({ op: 'subscribe', topic, type, throttle_rate: throttle, queue_length: 1 });
    },
    publish(topic, type, msg) {
      if (!topic || !this.open) return;
      if (!this.adv[topic]) { this.send({ op: 'advertise', topic, type }); this.adv[topic] = true; }
      this.send({ op: 'publish', topic, msg });
    },
    call(service, type, args) { this.send({ op: 'call_service', service, type, args, id: 'cgr_' + (++this.calls) }); },
  };

  /* ---------- frames: website (dashboard internal, y down) <-> robot map ---------- */
  const yawOf = q => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  const quat = yaw => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });
  function toRobot(x, y, th) {                        // internal → robot map
    const wx = x, wy = WORLD_H - y, c = Math.cos(MT.yaw), s = Math.sin(MT.yaw);
    return { x: MT.x0 + MT.scale * (c * wx - s * wy), y: MT.y0 + MT.scale * (s * wx + c * wy), yaw: -th + MT.yaw };
  }
  function fromRobot(rx, ry, ryaw) {                  // robot map → internal
    const dx = (rx - MT.x0) / MT.scale, dy = (ry - MT.y0) / MT.scale, c = Math.cos(MT.yaw), s = Math.sin(MT.yaw);
    const wx = c * dx + s * dy, wy = -s * dx + c * dy;
    return { x: wx, y: WORLD_H - wy, th: wrap(-(ryaw - MT.yaw)) };
  }

  /* ---------- robot state ---------- */
  const R = { live: false, haveAmcl: false, goalSent: 0, knownGoals: new Set(), ourGoal: null, lastCmd: 0, cam: null, camAt: 0, lastTft: '' };

  function setPose(p, fromAmcl) {
    if (fromAmcl) R.haveAmcl = true; else if (R.haveAmcl) return;   // odom only until localization starts
    const q = fromRobot(p.position.x, p.position.y, yawOf(p.orientation));
    S.pose.x = q.x; S.pose.y = q.y; S.pose.th = q.th;
  }

  function onConnect() {
    R.live = true;
    setChip('ROBOT · LIVE', '');
    if (S.nav) { S.nav = null; setNavState('IDLE'); renderNavUI(); }
    S.trail = []; S.v = S.w = 0;
    refreshTFT();
    console.info('[robot] connected to', URL_);
  }
  function onDisconnect() {
    R.live = false; R.haveAmcl = false;
    setChip('ROBOT · OFFLINE → SIM', 'red');
    if (S.nav) { S.nav = null; setNavState('LINK LOST', 'err'); renderNavUI(); refreshTFT(); }
    console.warn('[robot] connection lost, simulation takes over');
  }

  /* ---------- subscriptions ---------- */
  bridge.subscribe(TP.pose, 'geometry_msgs/msg/PoseWithCovarianceStamped', m => setPose(m.pose.pose, true));
  bridge.subscribe(TP.odom, 'nav_msgs/msg/Odometry', m => {
    setPose(m.pose.pose, false);
    S.v = m.twist.twist.linear.x; S.w = -m.twist.twist.angular.z;
  }, 100);
  bridge.subscribe(TP.scan, 'sensor_msgs/msg/LaserScan', m => {
    const out = []; let min = null;
    const step = Math.max(1, Math.round(m.ranges.length / 180));
    for (let i = 0; i < m.ranges.length; i += step) {
      const r = m.ranges[i];
      if (typeof r !== 'number' || !isFinite(r) || r < m.range_min || r > m.range_max) continue;
      const a = -(m.angle_min + i * m.angle_increment), rw = r / MT.scale;
      out.push({ a, r: rw });
      if (!min || rw < min.r) min = { a, r: rw };
    }
    S.scan = out; S.scanMin = min;
  }, 200);
  bridge.subscribe(TP.battery, 'sensor_msgs/msg/BatteryState', m => {
    let p = m.percentage;
    if (!(p >= 0)) p = (m.voltage - 9.6) / (12.6 - 9.6);          // TurtleBot3 LiPo 3S
    S.battDrive = Math.max(0, Math.min(100, p <= 1 ? p * 100 : p));
  }, 2000);
  bridge.subscribe(TP.camera, 'sensor_msgs/msg/CompressedImage', m => {
    const img = new Image();
    img.onload = () => { R.cam = img; R.camAt = performance.now(); };
    img.src = `data:image/${/png/i.test(m.format) ? 'png' : 'jpeg'};base64,${m.data}`;
  }, 100);
  bridge.subscribe(TP.navStatus, 'action_msgs/msg/GoalStatusArray', m => {
    const list = m.status_list || [];
    if (!list.length) return;
    const id = st => (st.goal_info.goal_id.uuid || []).join(',');
    if (R.goalSent && !R.ourGoal) {                    // first goal we haven't seen before = the one we sent
      const fresh = list.find(st => !R.knownGoals.has(id(st)));
      if (fresh) R.ourGoal = id(fresh);
    }
    const mine = list.find(st => id(st) === R.ourGoal);
    if (!mine || !S.nav || S.nav.phase === 'DONE') return;
    if (mine.status === 4) { R.goalSent = 0; arrive(); }                         // SUCCEEDED
    else if (mine.status === 6) {                                                // ABORTED
      R.goalSent = 0; S.nav = null; setNavState('FAILED', 'err'); renderNavUI(); refreshTFT(); speak(t('say_noplan'));
    } else if (mine.status === 5 && performance.now() - R.goalSent > 1500) {    // CANCELED elsewhere
      R.goalSent = 0; S.nav = null; setNavState('IDLE'); renderNavUI(); refreshTFT();
    }
  });

  /* ---------- outputs ---------- */
  function cmdVel(v, w) {
    const tw = { linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w } };
    if (C.cmdVelStamped) bridge.publish(TP.cmdVel, 'geometry_msgs/msg/TwistStamped', { header: { frame_id: 'base_link', stamp: { sec: 0, nanosec: 0 } }, twist: tw });
    else bridge.publish(TP.cmdVel, 'geometry_msgs/msg/Twist', tw);
  }
  function cancelAll() {
    bridge.call(TP.cancel, 'action_msgs/srv/CancelGoal', { goal_info: { goal_id: { uuid: new Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } } });
  }

  Object.assign(ROBOT, {
    live: () => R.live,

    // Replaces the simulated motion while connected: the pose comes from the robot.
    step(dt) {
      const now = performance.now();
      if (S.estop || S.mode === 'MANUAL') {
        let v = 0, w = 0;
        if (!S.estop) {
          const d = S.held;
          v = d === 'up' ? VMAX : d === 'down' ? -VMAX * 0.6 : 0;
          w = d === 'left' ? 1.2 : d === 'right' ? -1.2 : 0;     // ROS: positive = turn left
        }
        if (now - R.lastCmd > 100) { cmdVel(v, w); R.lastCmd = now; }   // 10 Hz while in control
      }
      const N = S.nav, P = S.pose;
      if (N && N.phase === 'DRIVE') {                  // advance the drawn route / turn-by-turn by proximity
        while (N.wp < N.path.length - 1 && Math.hypot(N.path[N.wp].x - P.x, N.path[N.wp].y - P.y) < 0.12) N.wp++;
      }
      const lt = S.trail[S.trail.length - 1];
      if (!lt || Math.hypot(lt.x - P.x, lt.y - P.y) > 0.02) { S.trail.push({ x: P.x, y: P.y }); if (S.trail.length > 600) S.trail.shift(); }
    },

    sendGoal(key, x, y, th) {
      const g = (C.goals && C.goals[key]) || toRobot(x, y, th);
      R.ourGoal = null; R.goalSent = performance.now();
      bridge.publish(TP.goal, 'geometry_msgs/msg/PoseStamped', {
        header: { frame_id: C.frame || 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: { position: { x: g.x, y: g.y, z: 0 }, orientation: quat(g.yaw) },
      });
      console.info(`[robot] goal ${key}: x=${g.x.toFixed(2)} y=${g.y.toFixed(2)} yaw=${g.yaw.toFixed(2)}`);
    },
    cancelGoal() { if (!R.live) return; R.goalSent = 0; cancelAll(); cmdVel(0, 0); },
    estop(on) { if (!R.live) return; if (on) { R.goalSent = 0; cancelAll(); } cmdVel(0, 0); },

    tft(info) {
      const s = JSON.stringify(info);
      if (s === R.lastTft) return;
      R.lastTft = s;
      bridge.publish(TP.tft, 'std_msgs/msg/String', { data: s });
    },
    say(text, lang) { if (C.speakOnRobot) bridge.publish(TP.say, 'std_msgs/msg/String', { data: JSON.stringify({ text, lang }) }); },

    // Live camera instead of the simulated view. Returns true when it drew.
    drawCamera(g, W, H) {
      if (!R.live || !TP.camera) return false;
      g.fillStyle = '#000'; g.fillRect(0, 0, W, H);
      const fresh = R.cam && performance.now() - R.camAt < 3000;
      if (fresh) {
        const k = Math.min(W / R.cam.width, H / R.cam.height), w = R.cam.width * k, h = R.cam.height * k;
        g.drawImage(R.cam, (W - w) / 2, (H - h) / 2, w, h);
      }
      g.font = '500 11px "IBM Plex Mono", monospace'; g.textBaseline = 'alphabetic';
      g.fillStyle = fresh ? '#3FCF7F' : '#F2B33D';
      g.fillText(fresh ? `● LIVE  ${TP.camera}` : `waiting for ${TP.camera}…`, 14, H - 14);
      for (const [id, v] of [['cv-target', 'live'], ['cv-conf', '—'], ['cv-ex', '—'], ['cv-h', '—'], ['cv-steer', 'ROBOT']]) {
        const el = document.getElementById(id); if (el.textContent !== v) el.textContent = v;
      }
      return true;
    },
  });

  // Learn which goals already exist before we send one (so a stale SUCCEEDED isn't mistaken for ours).
  const origStatusCb = bridge.subs[TP.navStatus] && bridge.subs[TP.navStatus].cb;
  if (origStatusCb) bridge.subs[TP.navStatus].cb = m => {
    if (!R.goalSent) for (const st of m.status_list || []) R.knownGoals.add((st.goal_info.goal_id.uuid || []).join(','));
    origStatusCb(m);
  };

  bridge.connect();
})();
