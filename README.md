# CGR-1 · Campus Guide Robot

Website and operator dashboard for an autonomous, multilingual (Korean / English) campus tour guide robot built on a TurtleBot 3 Burger with ROS 2, Nav2, slam_toolbox and YOLOv8.

The repo holds both halves: the website (product page, dashboard, Gemini guide server) and the robot code in [`robot/`](robot/README.md).

## Pages

| File | What it is |
| --- | --- |
| `index.html` | Product page: concept, how it works, navigation, interface, hardware, team |
| `dashboard.html` | Console: street-map style campus map with turn-by-turn directions, camera / landmark detection, the Gil voice assistant, TFT mirror, manual drive, E-Stop. Simulation by default, real robot with one switch |

No build step. Open `index.html` in a browser, or serve the folder:

```bash
python3 -m http.server 8000
```

## The guide: Gil (길이)

The robot talks as **Gil** (길이, short for 길잡이, "guide"), a friendly student-guide persona. Visitors can ask for directions *or* ask tour-guide questions: building history, opening hours, what's inside, Wi-Fi, parking, shuttle buses, and so on. Follow-up questions work ("When was the library built?" → "What time does it close?").

Answers come only from **`data/campus.json`**. The sample file is placeholder data for a fictional "Sample University": **replace it with your campus's real facts** before demoing. The server reloads the file automatically when you save it.

### Run the guide server

```bash
pip install -r server/requirements.txt
cp .env.example .env                # then put your key in .env (it is git-ignored)
python server/app.py                # open http://localhost:8000/dashboard.html
```

- `GEMINI_MODEL` is the list of models tried in order (default `gemini-3.1-flash-lite,gemini-3.5-flash-lite,gemini-3.8-flash`). If one is busy, retired or takes longer than 10 s, the next one answers. The bigger free-tier models are often overloaded, so the fast "lite" models go first.
- No key, no internet, or a Gemini error: it falls back to keyword matching over the same knowledge base, so the robot never goes silent.
- `POST /api/ask` with `{"text", "lang", "history"}` returns `{"intent": "navigate"|"answer"|"clarify", "target", "lang", "response"}`. The robot's onboard voice node (Mode A) can call the same endpoint.
- Change the persona (name, personality, greetings) in the `persona` block of `campus.json`.

Without the server (for example on GitHub Pages) the dashboard still answers campus questions: it loads `data/campus.json` in the browser and uses the same keyword matching. Opened as a plain local file it can only give directions, because browsers block reading the JSON file there.

## Simulation vs. real robot: one line

The dashboard runs a built-in simulation by default, so the demo works with no robot at all. Connecting the real TurtleBot is **one line** in `js/robot-config.js`:

```js
enabled: false,   // ← change to true to drive the real robot
```

or, without editing anything, open `dashboard.html?robot=on` (or `?robot=ws://<pi-ip>:9090`).

When it's on, the dashboard talks to the robot through **rosbridge** (plain WebSocket JSON, no extra libraries) and:

| Website | ROS 2 topic / service | |
| --- | --- | --- |
| Directions / "take me to…" | `/goal_pose` (PoseStamped) | Nav2 drives, like the RViz "Nav2 Goal" button |
| Arrival, failure | `/navigate_to_pose/_action/status` | "Arrived" only when Nav2 reports success |
| Exit, E-STOP | `/navigate_to_pose/_action/cancel_goal`, `/cmd_vel` = 0 | |
| Manual drive pad / WASD | `/cmd_vel` at 10 Hz | |
| Blue dot | `/amcl_pose` (`/odom` until AMCL starts) | |
| LiDAR layer, battery, camera | `/scan`, `/battery_state`, `/image_raw/compressed` | |
| Onboard TFT panel | `/cgr/tft` → `robot/scripts/cgr_display_node.py` | shown on the robot's screen |
| Gil's replies | `/cgr/say` → `robot/scripts/cgr_speaker_node.py` | spoken on the robot |

The robot chip in the top bar shows the link: **ROBOT · LIVE**, or **ROBOT · OFFLINE → SIM** (it keeps the simulation running and retries every 3 s, so a dropped link never breaks the demo).

Before the first real run, line the two maps up: set `mapTransform` and/or measure each building's goal pose on the robot (`goals` in `js/robot-config.js`). Robot-side setup, run order and goal calibration are in **[`robot/README.md`](robot/README.md)**.

> Serve the dashboard over **http** (`python server/app.py` → `http://localhost:8000`) when the robot is on. A page on GitHub Pages (https) can't open the robot's `ws://` connection.

Keyboard: `W A S D` / arrows to drive (manual mode), `Space` stop, `Esc` E-Stop.

## Structure

```
index.html
dashboard.html
css/site.css        product page styles
css/dashboard.css   console styles
js/dashboard.js     simulation, planner, UI logic
js/robot-config.js  robot on/off switch, topics, map alignment, goal poses
js/robot.js         rosbridge link (inactive unless switched on)
server/app.py       Flask server: website + /api/ask (Gemini + fallback)
data/campus.json    persona + everything the guide knows
robot/              code that runs on the TurtleBot (see robot/README.md)
  launch/cgr_web.launch.py        rosbridge + screen + speaker nodes
  scripts/cgr_display_node.py     website status on the 2.0" ST7789 TFT
  scripts/cgr_speaker_node.py     Gil's replies on the USB speaker (gTTS / espeak-ng)
  scripts/calibrate_camera.py     headless camera calibration
  scripts/tag_quicktest.py        AprilTag ID check
  scripts/display_test.py         TFT colour + text test
  scripts/display_scan.py         TFT SPI speed / mode finder
```
