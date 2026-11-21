import time, spidev
import RPi.GPIO as GPIO
DC, RST = 24, 25
W, H = 240, 320
GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM)
GPIO.setup(DC, GPIO.OUT); GPIO.setup(RST, GPIO.OUT)
spi = spidev.SpiDev(); spi.open(0, 0)

def cmd(c, data=None):
    GPIO.output(DC, 0); spi.writebytes([c])
    if data:
        GPIO.output(DC, 1); spi.writebytes2(bytes(data))

def init():
    GPIO.output(RST, 1); time.sleep(0.05)
    GPIO.output(RST, 0); time.sleep(0.05)
    GPIO.output(RST, 1); time.sleep(0.15)
    cmd(0x01); time.sleep(0.15); cmd(0x11); time.sleep(0.12)
    cmd(0x3A, [0x55]); cmd(0x36, [0x00]); cmd(0x21); cmd(0x13)
    cmd(0x29); time.sleep(0.05)

def fill(hi, lo):
    cmd(0x2A, [0, 0, 0, 239]); cmd(0x2B, [0, 0, 1, 63]); cmd(0x2C)
    GPIO.output(DC, 1); spi.writebytes2(bytes([hi, lo]) * (W * H))

for speed in (1000000, 8000000):
    for mode in (0, 3):
        spi.max_speed_hz = speed; spi.mode = mode
        print(f"Trying {speed//1000000} MHz, mode {mode} - watch the screen")
        init(); fill(0xF8, 0x00)
        time.sleep(3)
print("Finished")
