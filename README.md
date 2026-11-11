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

## Dashboard: simulation vs. real robot

The dashboard runs a built-in simulation (A* on a 0.05 m occupancy grid, simulated LiDAR and camera) so it works without hardware. To connect the real robot, edit the top of `js/dashboard.js`:

- `ROBOT.sendGoal / cancelGoal / cmdVel / estop`: replace with roslibjs calls over rosbridge (`/navigate_to_pose`, `/cmd_vel`).
- `CONFIG.intentEndpoint`: set to your Flask/FastAPI route (e.g. `/api/intent`) that returns `{ "target": "...", "response": "..." }` from the LLM.
- `B` table: building rectangles and goal poses in metres. Replace with coordinates from your saved map.

Keyboard: `W A S D` / arrows to drive (manual mode), `Space` stop, `Esc` E-Stop.

## Structure

```
index.html
dashboard.html
css/site.css        product page styles
css/dashboard.css   console styles
js/dashboard.js     simulation, planner, UI logic
```
