import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json

MODEL_PATH = "/home/zajac/Documents/David/vosk-model-small-en-us-0.15"

q = queue.Queue()

def callback(indata, frames, time, status):
    q.put(bytes(indata))

def main():
    print("Loading model...")
    model = Model(MODEL_PATH)

    # Use PulseAudio default device
    device = None  # or set to your device index explicitly, e.g. 9

    # Get default sample rate
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
                print("Recognized:", res.get("text", ""))
            else:
                pass  # partial results are ignored for simplicity

if __name__ == "__main__":
    main()
