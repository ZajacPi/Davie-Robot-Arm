import evdev

from evdev import InputDevice, categorize, ecodes 
#for testing what device number you have, type in console:
# python3 -m venv .venv
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

for device in devices:
    print(device.path, device.name, device.phys)

device = InputDevice('/dev/input/event9')
print("Listening for inputs...")

for event in device.read_loop():
    if event.type == ecodes.EV_KEY:
        button= categorize(event)
        print(f"Button {button.keycode} {'pressed ' if button.keystate else 'released'} ")
