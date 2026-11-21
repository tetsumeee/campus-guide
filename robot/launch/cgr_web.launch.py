"""
Starts everything the website needs on the robot (Raspberry Pi):
  - rosbridge websocket on port 9090 (the dashboard connects here)
  - cgr_display_node: website status on the 2.0" TFT
  - cgr_speaker_node: Gil's replies on the USB speaker

Run AFTER the usual robot bringup + Nav2 with your saved map:
    ros2 launch robot/launch/cgr_web.launch.py
    ros2 launch robot/launch/cgr_web.launch.py display:=false speaker:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")


def generate_launch_description():
    rosbridge = os.path.join(get_package_share_directory("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml")
    return LaunchDescription([
        DeclareLaunchArgument("display", default_value="true", description="Drive the TFT screen"),
        DeclareLaunchArgument("speaker", default_value="true", description="Speak replies on the robot"),
        DeclareLaunchArgument("port", default_value="9090"),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge),
                                 launch_arguments={"port": LaunchConfiguration("port")}.items()),
        ExecuteProcess(cmd=["python3", os.path.join(SCRIPTS, "cgr_display_node.py")], output="screen",
                       condition=IfCondition(LaunchConfiguration("display"))),
        ExecuteProcess(cmd=["python3", os.path.join(SCRIPTS, "cgr_speaker_node.py")], output="screen",
                       condition=IfCondition(LaunchConfiguration("speaker"))),
    ])
