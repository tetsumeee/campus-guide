# CGR-1 · Campus Guide Robot

Website and operator dashboard for an autonomous, multilingual (Korean / English) campus tour guide robot built on a TurtleBot 3 Burger with ROS 2, Nav2, slam_toolbox and YOLOv8.

## Pages

| File | What it is |
| --- | --- |
| `index.html` | Product page: concept, how it works, navigation, interface, hardware, team |
| `dashboard.html` | Demo console: street-map style campus map with turn-by-turn directions, camera / landmark detection, the Gil voice assistant, TFT mirror, manual drive, E-Stop |

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
data/campus.json   persona + everything the guide knows
```
