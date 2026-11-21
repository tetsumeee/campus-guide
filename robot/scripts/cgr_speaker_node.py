#!/usr/bin/env python3
"""
CGR-1 speaker: says Gil's replies out loud on the robot's USB speaker.

Subscribes to /cgr/say (std_msgs/String, JSON from the dashboard):
    {"text": "Follow me to the Campus Cafe!", "lang": "en"}

Uses gTTS (natural Google voice, needs internet) and falls back to espeak-ng
(offline, robotic) when there's no connection.

    sudo apt install mpg123 espeak-ng
    pip install gTTS
    python3 robot/scripts/cgr_speaker_node.py
"""
import json
import os
import shutil
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeakerNode(Node):
    def __init__(self):
        super().__init__("cgr_speaker")
        self.declare_parameter("topic", "/cgr/say")
        self.player = None
        self.lock = threading.Lock()
        self.create_subscription(String, self.get_parameter("topic").value, self.on_msg, 10)
        self.get_logger().info("Speaking messages from " + self.get_parameter("topic").value)

    def on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            data = {"text": msg.data}
        text, lang = data.get("text", "").strip(), data.get("lang", "en")
        if text:
            threading.Thread(target=self.say, args=(text, lang), daemon=True).start()

    def stop_current(self):
        if self.player and self.player.poll() is None:
            self.player.terminate()

    def say(self, text, lang):
        with self.lock:                      # newest reply interrupts the previous one
            self.stop_current()
            try:
                from gtts import gTTS
                path = os.path.join(tempfile.gettempdir(), "cgr_say.mp3")
                gTTS(text=text, lang="ko" if lang == "ko" else "en").save(path)
                self.player = subprocess.Popen(["mpg123", "-q", path])
                return
            except Exception as e:           # no internet / gTTS missing
                self.get_logger().warn(f"gTTS unavailable ({e}); using espeak-ng")
            if shutil.which("espeak-ng"):
                self.player = subprocess.Popen(["espeak-ng", "-v", "ko" if lang == "ko" else "en-us", text])
            else:
                self.get_logger().error("No TTS available: install gTTS + mpg123, or espeak-ng")


def main():
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_current()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
