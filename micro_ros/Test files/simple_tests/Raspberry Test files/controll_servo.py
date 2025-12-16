import time
import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from evdev import InputDevice, categorize, ecodes 

#-------- controller setup ---------------
#for testing what device number you have, type in console:
# python -m evdev.evtest
device = InputDevice('/dev/input/event7')
BTN_A = ecodes.BTN_B
BTN_B = ecodes.BTN_A

button_state = {BTN_A: 0, BTN_B: 0}
#-------- Servo Setup ------------------------
i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
pca = PCA9685(i2c)
pca.frequency = 50
servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)
#set initial angle
angle = 90
servo.angle = angle


print("Listening for inputs...")
for event in device.read_loop():
    if event.type == ecodes.EV_KEY:
        # button = categorize(event)
        # if event.value == 1:
            # if event.code == BTN_A:
            #     print(f"A button pressed! ")
            # if event.code == BTN_B:
            #     print(f"B button pressed! ")
        if event.code in button_state:
            button_state[event.code] = event.value
            # print(event.value)

if button_state[BTN_A] == 1  :
    angle = max(0, angle-1)
    servo.angle = angle
    print(f"Servo moved to {angle}")
    # time.sleep(0.1)
        