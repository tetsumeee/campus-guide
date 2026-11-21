/* ==========================================================================
   Robot connection settings
   --------------------------------------------------------------------------
   enabled: false  → dashboard runs its built-in simulation (safe for demos)
   enabled: true   → dashboard drives the real TurtleBot through rosbridge

   You can also switch it on without editing this file by opening
   dashboard.html?robot=on   (or ?robot=ws://192.168.0.42:9090 for another address)

   If the robot can't be reached, the dashboard keeps running the simulation
   and keeps retrying in the background, so the demo never breaks.
   ========================================================================== */
window.ROBOT_CONFIG = {
  enabled: false,                         // ← THE ONE LINE: true = connect to the robot

  url: 'ws://turtlebot.local:9090',       // rosbridge on the Pi (use its IP if .local doesn't resolve)

  // ROS topics (TurtleBot3 + Nav2 defaults)
  topics: {
    goal: '/goal_pose',                   // geometry_msgs/PoseStamped  (same as RViz "Nav2 Goal")
    navStatus: '/navigate_to_pose/_action/status',
    cancel: '/navigate_to_pose/_action/cancel_goal',
    pose: '/amcl_pose',                   // geometry_msgs/PoseWithCovarianceStamped (Nav2 localization)
    odom: '/odom',                        // used for the pose until AMCL publishes
    cmdVel: '/cmd_vel',
    scan: '/scan',
    battery: '/battery_state',
    camera: '/image_raw/compressed',      // sensor_msgs/CompressedImage; '' to keep the simulated camera
    tft: '/cgr/tft',                      // std_msgs/String JSON → robot/scripts/cgr_display_node.py
    say: '/cgr/say',                      // std_msgs/String JSON → robot/scripts/cgr_speaker_node.py
  },
  cmdVelStamped: false,                   // ROS 2 Jazzy TurtleBot3 uses TwistStamped: set true there
  frame: 'map',

  // How the website map lines up with the robot's SLAM map.
  // Website coordinates: origin at the bottom-left corner of the 4.0 × 3.0 m map, y up.
  // robot_x = x0 + scale * (cos(yaw)*web_x - sin(yaw)*web_y), likewise for y.
  mapTransform: { x0: 0, y0: 0, yaw: 0, scale: 1 },

  // Exact goal poses measured on the real robot (map frame, metres / radians).
  // Park the robot at each building and run:  ros2 topic echo /amcl_pose --once
  // Any building left as null uses the website map position through mapTransform.
  goals: {
    library: null,       // e.g. { x: 0.68, y: 1.90, yaw: 1.57 }
    cafe: null,
    union: null,
    engineering: null,
    business: null,
    restroom: null,
  },

  speakOnRobot: true,    // also send Gil's replies to the robot's speaker (/cgr/say)
};
