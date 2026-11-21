#!/usr/bin/env python3
"""
CGR-1 onboard screen: shows what the website's "Onboard TFT" panel shows.

Subscribes to /cgr/tft (std_msgs/String, JSON from the dashboard):
    {"main": "Campus Cafe", "sub": "Follow me", "state": "MOVING", "mode": "AUTO", "batt": 84, "lang": "en"}
and draws it on the 2.0" ST7789 SPI screen (240x320), using the same wiring
and init sequence as display_test.py (DC=GPIO24, RST=GPIO25, SPI0 CE0).

    python3 robot/scripts/cgr_display_node.py
    python3 robot/scripts/cgr_display_node.py --ros-args -p spi_hz:=8000000 -p spi_mode:=0

If display_scan.py showed a different working speed/mode, pass it as above.
Korean text needs a CJK font:  sudo apt install fonts-noto-cjk
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PIL import Image, ImageDraw, ImageFont

try:
    import spidev
    import RPi.GPIO as GPIO
    HW = True
except ImportError:          # lets you test the drawing on a laptop: it saves tft_preview.png instead
    HW = False

W, H = 240, 320
COLORS = {"READY": (63, 207, 127), "MOVING": (255, 79, 0), "ARRIVED": (66, 133, 244),
          "MANUAL": (242, 179, 61), "E-STOP": (229, 55, 43)}
FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",       # Korean + English
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",      # English only
]


def load_font(size):
    for p in FONT_PATHS:
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


class ST7789:
    """Same commands as display_test.py, wrapped in a class."""

    def __init__(self, dc, rst, hz, mode):
        self.dc, self.rst = dc, rst
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dc, GPIO.OUT)
        GPIO.setup(rst, GPIO.OUT)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = hz
        self.spi.mode = mode
        GPIO.output(rst, 1); time.sleep(0.05)
        GPIO.output(rst, 0); time.sleep(0.05)
        GPIO.output(rst, 1); time.sleep(0.15)
        self.cmd(0x01); time.sleep(0.15)     # software reset
        self.cmd(0x11); time.sleep(0.12)     # wake up
        self.cmd(0x3A, [0x55])               # 16-bit colour
        self.cmd(0x36, [0x00])               # orientation
        self.cmd(0x21)                       # inversion on (IPS)
        self.cmd(0x13)                       # normal mode
        self.cmd(0x29); time.sleep(0.05)     # display on

    def cmd(self, c, data=None):
        GPIO.output(self.dc, 0)
        self.spi.writebytes([c])
        if data:
            GPIO.output(self.dc, 1)
            self.spi.writebytes2(bytes(data))

    def show(self, img):
        px = img.convert("RGB").resize((W, H)).tobytes()
        try:                                   # fast path
            import numpy as np
            a = np.frombuffer(px, dtype=np.uint8).reshape(-1, 3).astype(np.uint16)
            v = ((a[:, 0] & 0xF8) << 8) | ((a[:, 1] & 0xFC) << 3) | (a[:, 2] >> 3)
            buf = v.astype(">u2").tobytes()
        except ImportError:                    # same loop as display_test.py
            buf = bytearray(W * H * 2)
            j = 0
            for i in range(0, len(px), 3):
                v = ((px[i] & 0xF8) << 8) | ((px[i + 1] & 0xFC) << 3) | (px[i + 2] >> 3)
                buf[j] = v >> 8; buf[j + 1] = v & 0xFF; j += 2
        self.cmd(0x2A, [0, 0, (W - 1) >> 8, (W - 1) & 0xFF])
        self.cmd(0x2B, [0, 0, (H - 1) >> 8, (H - 1) & 0xFF])
        self.cmd(0x2C)
        GPIO.output(self.dc, 1)
        self.spi.writebytes2(buf)


def wrap_text(draw, text, font, width):
    """Wrap by words, or by characters for Korean (no spaces needed)."""
    lines, line = [], ""
    for ch in text:
        trial = line + ch
        if draw.textlength(trial, font=font) <= width:
            line = trial
            continue
        cut = line.rfind(" ")
        if cut > 0 and ch != " ":
            lines.append(line[:cut]); line = line[cut + 1:] + ch
        else:
            lines.append(line); line = ch.lstrip()
    if line:
        lines.append(line)
    return lines


def render(info):
    img = Image.new("RGB", (W, H), (0, 0, 0))
    d = ImageDraw.Draw(img)
    small, mid, big = load_font(14), load_font(17), load_font(30)
    state = info.get("state", "READY")
    accent = COLORS.get(state, (255, 255, 255))
    # header
    d.rectangle((0, 0, W, 34), fill=(20, 21, 24))
    d.text((10, 8), "CGR-1", font=small, fill=(220, 220, 220))
    d.text((W // 2, 8), info.get("mode", "AUTO"), font=small, fill=accent, anchor="ma")
    d.text((W - 10, 8), f"{info.get('batt', '--')}%", font=small, fill=(220, 220, 220), anchor="ra")
    # main + sub text, centred
    main = wrap_text(d, info.get("main", ""), big, W - 24)[:4]
    sub = wrap_text(d, info.get("sub", ""), mid, W - 24)[:3]
    total = len(main) * 38 + 10 + len(sub) * 24
    y = 34 + (H - 34 - 48 - total) // 2
    for line in main:
        d.text((W // 2, y), line, font=big, fill=(255, 255, 255), anchor="ma"); y += 38
    y += 10
    for line in sub:
        d.text((W // 2, y), line, font=mid, fill=(170, 170, 170), anchor="ma"); y += 24
    # footer with state colour
    d.rectangle((0, H - 44, W, H), fill=accent)
    d.text((W // 2, H - 31), state, font=mid, fill=(0, 0, 0), anchor="ma")
    return img


class DisplayNode(Node):
    def __init__(self):
        super().__init__("cgr_display")
        self.declare_parameter("spi_hz", 1000000)     # display_test.py setting
        self.declare_parameter("spi_mode", 3)
        self.declare_parameter("dc", 24)
        self.declare_parameter("rst", 25)
        self.declare_parameter("topic", "/cgr/tft")
        p = lambda n: self.get_parameter(n).value
        self.lcd = ST7789(p("dc"), p("rst"), p("spi_hz"), p("spi_mode")) if HW else None
        self.pending, self.lock = None, threading.Lock()
        self.create_subscription(String, p("topic"), self.on_msg, 10)
        self.on_msg(String(data=json.dumps({"main": "Hi, I'm Gil!", "sub": "Ask me about campus", "state": "READY"})))
        threading.Thread(target=self.draw_loop, daemon=True).start()
        self.get_logger().info(f"Showing {p('topic')} on the screen" + ("" if HW else " (no SPI: writing tft_preview.png)"))

    def on_msg(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            info = {"main": msg.data}
        with self.lock:
            self.pending = info           # only the newest message is drawn

    def draw_loop(self):
        while rclpy.ok():
            with self.lock:
                info, self.pending = self.pending, None
            if info is None:
                time.sleep(0.05)
                continue
            img = render(info)
            if self.lcd:
                self.lcd.show(img)        # ~1 s per frame at 1 MHz; faster SPI = faster updates
            else:
                img.save("tft_preview.png")


def main():
    rclpy.init()
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if HW:
            GPIO.cleanup()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
