import rclpy
from rclpy.node import Node
from speech_recognition.msg import MovementCommand

import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json


MODEL_PATH = "/home/zajac/Documents/David/vosk-model-small-en-us-0.15"
q = queue.Queue()


# Same text2int you already have  ---------------------------------------------
def text2int(textnum, numwords={}):
    if not textnum:
        raise Exception("Empty number")

    if not numwords:
        units = [
            "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
            "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
            "sixteen", "seventeen", "eighteen", "nineteen",
        ]

        tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]
        scales = ["hundred", "thousand", "million", "billion", "trillion"]

        numwords["and"] = (1, 0)

        for idx, word in enumerate(units): numwords[word] = (1, idx)
        for idx, word in enumerate(tens): numwords[word] = (1, idx * 10)
        for idx, word in enumerate(scales): numwords[word] = (10 ** (idx * 3 or 2), 0)

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


# Command dictionary -----------------------------------------------------------
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



def parse_command(text):
    words = text.split()

    if len(words) < 3:
        return None

    if words[0] != "david":
        return None

    # last word = number
    try:
        number = text2int(words[-1])
    except Exception:
        return None

    command_text = " ".join(words[1:-1])

    if command_text not in commands:
        return None

    return commands[command_text](number)



# ----------------------- ROS2 NODE CLASS -------------------------------------

class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_commands_node")

        self.publisher = self.create_publisher(MovementCommand, "movement_cmd", 10)

        self.get_logger().info("Loading Vosk model...")
        self.model = Model(MODEL_PATH)

        device = None
        samplerate = int(sd.query_devices(device, "input")["default_samplerate"])
        self.recognizer = KaldiRecognizer(self.model, samplerate)

        self.stream = sd.RawInputStream(
            samplerate=samplerate,
            blocksize=8000,
            device=device,
            dtype='int16',
            channels=1,
            callback=self.audio_callback,
        )
        self.stream.start()

        self.timer = self.create_timer(0.01, self.process_audio)
        self.get_logger().info("Voice control node started. Say: 'david go up 10'")

    def audio_callback(self, indata, frames, time, status):
        q.put(bytes(indata))

    def process_audio(self):
        if q.empty():
            return

        data = q.get()
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").strip()

            if not text:
                return

            self.get_logger().info(f"Recognized: {text}")

            cmd = parse_command(text)
            if cmd:
                axis, value = cmd
                msg = MovementCommand(axis=axis, value=float(value))
                self.publisher.publish(msg)
                self.get_logger().info(f"Published command: axis={axis}, value={value}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
