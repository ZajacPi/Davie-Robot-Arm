import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json


MODEL_PATH = "/home/zajac/Documents/David/vosk-model-small-en-us-0.15"

q = queue.Queue()

def text2int(textnum, numwords={}):     
    # Source - https://stackoverflow.com/a
    # Posted by recursive, modified by community. See post 'Timeline' for change history
    # Retrieved 2025-11-23, License - CC BY-SA 3.0
    if not numwords:
      units = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
        "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
        "sixteen", "seventeen", "eighteen", "nineteen",
      ]

      tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]

      scales = ["hundred", "thousand", "million", "billion", "trillion"]

      numwords["and"] = (1, 0)

      for idx, word in enumerate(units):    numwords[word] = (1, idx)
      for idx, word in enumerate(tens):     numwords[word] = (1, idx * 10)
      for idx, word in enumerate(scales):   numwords[word] = (10 ** (idx * 3 or 2), 0)

    current = result = 0
    for word in textnum.split():
        if word not in numwords:
          raise Exception("Illegal word: " + word)

        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    return result + current

commands = {
    "go up":        lambda n: {"z":  n},
    "go down":      lambda n: {"z": -n},
    "go left":      lambda n: {"y": -n},
    "go right":     lambda n: {"y":  n},
    "go forward":   lambda n: {"x":  n},
    "go backward":  lambda n: {"x": -n},
    "pitch":        lambda n: {"pitch": n},
    "pitch minus":  lambda n: {"pitch":-n},
    "roll":         lambda n: {"roll":  n},
    "roll minus":   lambda n: {"roll": -n},
    "yaw":          lambda n: {"yaw":   n},
    "yaw minus":    lambda n: {"yaw":  -n},
}

def robot_controll(command, value):
    x = y = z = roll = pitch = yaw = 0
    if command in commands:
        action = commands[command](value)
        x += action.get("x", 0)
        y += action.get("y", 0)
        z += action.get("z", 0)
        roll += action.get("roll", 0)
        pitch += action.get("pitch", 0)
        yaw += action.get("yaw", 0)
    return [[x, y, z], [roll, pitch, yaw]]

def parse_command(text):
    words = text.split()
    if len(words) > 4:
        return None

    if words[0] != "david":
        return None  # no trigger word

    # Last word must be the number
    number_word = words[-1]
    try:
        number = text2int(number_word)
    except Exception:
        print("Could not parse number:", number_word)
        return None

    command_text = " ".join(words[1:-1])
    if command_text not in commands:
        print("Unknown command:", command_text)
        return None
    return commands[command_text](number)



def robot_controll(cmd_dict):
    # This is where you will publish to ROS2
    print("Robot command generated:", cmd_dict)
    return cmd_dict

def callback(indata, frames, time, status):
    q.put(bytes(indata))

def main():
    print("Loading model...")
    model = Model(MODEL_PATH)
    device = None  

    samplerate = int(sd.query_devices(device, "input")["default_samplerate"])

    recognizer = KaldiRecognizer(model, samplerate)

    with sd.RawInputStream(
        samplerate=samplerate,
        blocksize=8000,
        device=device,
        dtype='int16',
        channels=1,
        callback=callback,
    ):
        print("Listening... Ctrl+C to stop")

        while True:
            data = q.get()
            if recognizer.AcceptWaveform(data):
                res = json.loads(recognizer.Result())
                text = res.get("text", "").strip()
                if not text:
                    continue
                print("Recognized:", text)
               
                cmd = parse_command(text)
                if cmd:
                    robot_controll(cmd)                    
            else:
                pass  # partial results are ignored for simplicity

if __name__ == "__main__":
    main()
