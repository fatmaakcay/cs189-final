# Sample code for speech recognition
# Uses google speech engine API
# Network should be connected to the system


import pyaudio
import speech_recognition as sr

index = pyaudio.PyAudio().get_device_count() - 1
print index

r = sr.Recognizer()
for i in range(5):
    with sr.Microphone() as source:
    audio = r.listen(source)

try:
    print("You said " + r.recognize(audio))
except LookupError:
    print("Could not understand audio")