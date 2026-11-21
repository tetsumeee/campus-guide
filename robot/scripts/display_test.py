import time, spidev
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw, ImageFont

DC, RST = 24, 25          # Pin 18, Pin 22
W, H = 240, 320

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(DC, GPIO.OUT)
GPIO.setup(RST, GPIO.OUT)

spi = spidev.SpiDev()
spi.open(0, 0)            # CE0 = Pin 24
spi.max_speed_hz = 1000000
spi.mode = 3

def cmd(c, data=None):
    GPIO.output(DC, 0); spi.writebytes([c])
    if data:
        GPIO.output(DC, 1); spi.writebytes2(bytes(data))

# Hardware reset
GPIO.output(RST, 1); time.sleep(0.05)
GPIO.output(RST, 0); time.sleep(0.05)
GPIO.output(RST, 1); time.sleep(0.15)

# ST7789 init
cmd(0x01); time.sleep(0.15)   # software reset
cmd(0x11); time.sleep(0.12)   # wake up
cmd(0x3A, [0x55])             # 16-bit color
cmd(0x36, [0x00])             # orientation
cmd(0x21)                     # color inversion ON (needed for IPS)
cmd(0x13)                     # normal mode
cmd(0x29); time.sleep(0.05)   # display ON

def show(img):
    px = img.convert("RGB").resize((W, H)).tobytes()
    buf = bytearray(W * H * 2)
    j = 0
    for i in range(0, len(px), 3):
        v = ((px[i] & 0xF8) << 8) | ((px[i+1] & 0xFC) << 3) | (px[i+2] >> 3)
        buf[j] = v >> 8; buf[j+1] = v & 0xFF; j += 2
    cmd(0x2A, [0, 0, (W-1) >> 8, (W-1) & 0xFF])
    cmd(0x2B, [0, 0, (H-1) >> 8, (H-1) & 0xFF])
    cmd(0x2C)
    GPIO.output(DC, 1); spi.writebytes2(buf)

# Color test
for color in ["red", "green", "blue"]:
    show(Image.new("RGB", (W, H), color)); time.sleep(1)

# Text test
img = Image.new("RGB", (W, H), "navy")
d = ImageDraw.Draw(img)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
except OSError:
    font = ImageFont.load_default()
d.text((20, 130), "Hello", font=font, fill="white")
d.text((20, 165), "TurtleBot3!", font=font, fill="yellow")
d.rectangle((0, 0, W-1, H-1), outline="white", width=3)
show(img)
print("Done - check the screen")
time.sleep(15)
spi.close()
