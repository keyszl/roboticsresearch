import vosk
import pyaudio
import json
import pyttsx3
import sounddevice

class voskRecognizer():
    def __init__(self):
        model_path = "/home/student/vosk-model-small-en-us-0.15"
        model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(model, 16000)
        self.p = pyaudio.PyAudio()
        self.speaker = pyttsSpeaker()
    def getInput(self, text):
        if len(text) >= 1:
            self.speaker.outputSpeech(text)
        stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
        recognized_text = ""
        while len(recognized_text) == 0:
            data = stream.read(4096)#read in chunks of 4096 bytes
            if self.rec.AcceptWaveform(data):#accept waveform of input voice
                # Parse the JSON result and get the recognized text
                result = json.loads(self.rec.Result())
                recognized_text = result['text']
        return recognized_text
        stream.stop_stream()
        stream.close()

class pyttsSpeaker():
    def __init__(self):
        # Initialize the TTS engine
        self.speaker = pyttsx3.init()
        # Set properties (optional)
        self.speaker.setProperty('rate', 130)  # Speed of speech (words per minute)
        self.speaker.setProperty('volume', 1.0)  # Volume (0.0 to 1.0)
        self.speaker.setProperty('voice', 'English (America)')
    def outputSpeech(self, text):
        self.speaker.say(text) # Convert text to speech and play it
        self.speaker.runAndWait() # Wait for the speech to finish

if __name__ == '__main__':
    speechRecognizer = voskRecognizer()
    print(speechRecognizer.getInput("Ready for command"))
