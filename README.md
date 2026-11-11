# CGR-1 · Campus Guide Robot

Website and operator dashboard for an autonomous, multilingual (Korean / English) campus tour guide robot built on a TurtleBot 3 Burger with ROS 2, Nav2, slam_toolbox and YOLOv8.

## Pages

| File | What it is |
| --- | --- |
| `index.html` | Product page: concept, how it works, navigation, interface, hardware, team |
| `dashboard.html` | Operator console: map + planner, LiDAR, camera / landmark detection, voice + intent, TFT mirror, teleop, E-Stop, event log |

No build step. Open `index.html` in a browser, or serve the folder:

```bash
python3 -m http.server 8000
```

## The guide: Gil (길이)

The robot talks as **Gil** (길이, short for 길잡이, "guide"), a friendly student-guide persona. Visitors can ask for directions *or* ask tour-guide questions: building history, opening hours, what's inside, Wi-Fi, parking, shuttle buses, and so on. Follow-up questions work ("When was the library built?" → "What time does it close?").

Answers come only from **`server/knowledge/campus.json`**. The sample file is placeholder data for a fictional "Sample University": **replace it with your campus's real facts** before demoing. The server reloads the file automatically when you save it.

### Run the guide server

```bash
pip install -r server/requirements.txt
export GEMINI_API_KEY=your-key      # free key: https://aistudio.google.com/apikey
python server/app.py                # open http://localhost:8000/dashboard.html
```

- `GEMINI_MODEL` picks the model (default `gemini-flash-latest`).
- No key, no internet, or a Gemini error: it falls back to keyword matching over the same knowledge base, so the robot never goes silent.
- `POST /api/ask` with `{"text", "lang", "history"}` returns `{"intent": "navigate"|"answer"|"clarify", "target", "lang", "response"}`. The robot's onboard voice node (Mode A) can call the same endpoint.
- Change the persona (name, personality, greetings) in the `persona` block of `campus.json`.

When the dashboard is opened as a plain file or from GitHub Pages (no server), it falls back to directions only.

## Dashboard: simulation vs. real robot

The dashboard runs a built-in simulation (A* on a 0.05 m occupancy grid, simulated LiDAR and camera) so it works without hardware. To connect the real robot, edit the top of `js/dashboard.js`:

- `ROBOT.sendGoal / cancelGoal / cmdVel / estop`: replace with roslibjs calls over rosbridge (`/navigate_to_pose`, `/cmd_vel`).
- `CONFIG.intentEndpoint`: set automatically to `/api/ask` when the page is served by `server/app.py`.
- `B` table: building rectangles and goal poses in metres. Replace with coordinates from your saved map.

Keyboard: `W A S D` / arrows to drive (manual mode), `Space` stop, `Esc` E-Stop.

## Structure

```
index.html
dashboard.html
css/site.css        product page styles
css/dashboard.css   console styles
js/dashboard.js     simulation, planner, UI logic
server/app.py       Flask server: website + /api/ask (Gemini + fallback)
server/knowledge/campus.json   persona + everything the guide knows
```
