# Sample code for speech recognition
# Uses google speech engine API
# Network should be connected to the system


import pyaudio
import speech_recognition as sr

index = pyaudio.PyAudio().get_device_count() - 1
print index

#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class


# obtain audio from the microphone
r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say something!")
    audio = r.listen(source)

# recognize speech using Google Speech Recognition
try:
# for testing purposes, we're just using the default API key
# to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
# instead of `r.recognize_google(audio)`
    print("detected voices")
    print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))