#!/usr/bin/env python3
# Requires PyAudio and PySpeech.



import speech_recognition as sr
import pyaudio

pa = pyaudio.PyAudio()
chosen_device_index = -1
for x in xrange(0,pa.get_device_count()):
    info = pa.get_device_info_by_index(x)
    print pa.get_device_info_by_index(x)
    if info["name"] == "pulse":
        chosen_device_index = info["index"]
        print "Chosen index: ", chosen_device_index

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input_device_index=chosen_device_index, input=True, output=False)

stream.start_stream()

# Record Audio
r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say something!")
    audio = r.listen(source)

# Speech recognition using Google Speech Recognition
try:
    # for testing purposes, we're just using the default API key
    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    # instead of `r.recognize_google(audio)`
    print("You said: " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
