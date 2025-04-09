import evdev

from evdev import InputDevice, categorize, ecodes 
#for testing what device number you have, type in console:
# python3 -m venv .venv
device = InputDevice('/dev/input/event7')
print("Listening for inputs...")

for event in device.read_loop():
    if event.type == ecodes.EV_KEY:
        button= categorize(event)
        print(f"Button {button.keycode} {'pressed ' if button.keystate else 'released'} ")
