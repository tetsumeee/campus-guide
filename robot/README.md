# CGR-1 robot code

Everything that runs on the TurtleBot 3 Burger (Raspberry Pi 4, Ubuntu 22.04, ROS 2 Humble), plus how to link it to the website.

The website does **not** need any of this for the demo: it runs its own simulation until you switch the robot on in [`js/robot-config.js`](../js/robot-config.js).

## What's here

| File | What it does |
| --- | --- |
| `launch/cgr_web.launch.py` | Starts rosbridge (port 9090), the screen node and the speaker node |
| `scripts/cgr_display_node.py` | Shows the website's status (`/cgr/tft`) on the 2.0" ST7789 screen. Same wiring and init as `display_test.py` |
| `scripts/cgr_speaker_node.py` | Speaks Gil's replies (`/cgr/say`) on the USB speaker: gTTS online, espeak-ng offline |
| `scripts/calibrate_camera.py` | Headless camera calibration (8×6 inner corners, 25 mm squares), saves `~/.ros/camera_info/mmal_service_16.1.yaml` |
| `scripts/tag_quicktest.py` | Prints which AprilTag (36h11) IDs the camera sees on `/image_raw` |
| `scripts/display_test.py` | TFT colour + text test |
| `scripts/display_scan.py` | Tries SPI speeds / modes to find what the TFT accepts |

## One-time setup on the Pi

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-image-transport-plugins \
                 python3-spidev python3-rpi.gpio python3-numpy python3-pil \
                 fonts-noto-cjk mpg123 espeak-ng
pip install gTTS
```

`image-transport-plugins` makes the camera also publish `/image_raw/compressed`, which is what the dashboard shows. `fonts-noto-cjk` lets the screen show Korean.

## Run order

On the robot (each in its own terminal, `export TURTLEBOT3_MODEL=burger`):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

```bash
ros2 run v4l2_camera v4l2_camera_node        # or the camera node you already use; must publish /image_raw
```

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

```bash
ros2 launch robot/launch/cgr_web.launch.py
```

In RViz, give the robot its starting position once with **2D Pose Estimate**, so `/amcl_pose` starts publishing.

On the laptop:

1. In `js/robot-config.js` set `enabled: true` and `url: 'ws://<pi-ip>:9090'`. Or skip editing and open `dashboard.html?robot=ws://<pi-ip>:9090`.
2. Run `python server/app.py` and open `http://localhost:8000/dashboard.html` (http, not GitHub Pages).
3. The top bar should show **ROBOT · LIVE**. Tap a building → **Directions**.

To switch back to the simulation, set `enabled: false` again (or open the page without `?robot`).

## Line up the website map with the robot's map

The website map is a fixed 4.0 × 3.0 m drawing. Your SLAM map has its own origin, so tell the website where each building really is. Two options:

**A. Measure each goal (most accurate, recommended)**
Drive the robot (teleop) to the spot in front of each building where it should stop, facing the building, and run:

```bash
ros2 topic echo /amcl_pose --once
```

Put `position.x`, `position.y` and the heading into `goals` in `js/robot-config.js`. Heading: `yaw = 2 * atan2(orientation.z, orientation.w)`.

```js
goals: { library: { x: 0.42, y: 1.87, yaw: 1.57 }, cafe: { x: 1.60, y: 1.88, yaw: 1.57 }, ... }
```

**B. One transform for the whole map (quick)**
`mapTransform: { x0, y0, yaw, scale }` = where the website map's bottom-left corner sits in the robot map, its rotation, and real metres per website metre. This also moves the blue dot on the website, so set it roughly even if you use option A.

## Topics the website uses

| Topic | Type | Direction |
| --- | --- | --- |
| `/goal_pose` | geometry_msgs/PoseStamped | website → Nav2 |
| `/navigate_to_pose/_action/status` | action_msgs/GoalStatusArray | Nav2 → website (arrived / failed) |
| `/navigate_to_pose/_action/cancel_goal` | action_msgs/srv/CancelGoal | website → Nav2 (Exit, E-STOP) |
| `/cmd_vel` | geometry_msgs/Twist | website → robot (manual drive, E-STOP zero) |
| `/amcl_pose`, `/odom` | PoseWithCovarianceStamped, Odometry | robot → website (blue dot, speed) |
| `/scan`, `/battery_state`, `/image_raw/compressed` | LaserScan, BatteryState, CompressedImage | robot → website |
| `/cgr/tft`, `/cgr/say` | std_msgs/String (JSON) | website → screen / speaker nodes |

Topic names are all in `js/robot-config.js` if yours differ. On ROS 2 Jazzy, TurtleBot3 expects `TwistStamped` on `/cmd_vel`: set `cmdVelStamped: true`.

## Troubleshooting

- **ROBOT · OFFLINE → SIM**: rosbridge isn't reachable. Check the Pi's IP, that `cgr_web.launch.py` is running, and that laptop and Pi are on the same Wi-Fi.
- **ROBOT · NEEDS http:// PAGE**: the dashboard was opened over https (e.g. GitHub Pages). Use `http://localhost:8000`.
- **Blue dot doesn't move**: set the 2D Pose Estimate in RViz (AMCL only publishes after that); check `ros2 topic echo /amcl_pose`.
- **Goal sent but robot doesn't drive**: check Nav2 is active and `ros2 topic echo /goal_pose` shows the goal in the `map` frame.
- **Screen stays black**: run `display_scan.py`, then start the node with the speed/mode that worked: `python3 scripts/cgr_display_node.py --ros-args -p spi_hz:=8000000 -p spi_mode:=0`.
- **Screen updates slowly**: at 1 MHz a full frame takes about 1 s. A higher `spi_hz` that `display_scan.py` confirms makes it faster.
