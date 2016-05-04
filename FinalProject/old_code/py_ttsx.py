import pyttsx
engine = pyttsx.init()
engine.setProperty('rate', 180)

voices = engine.getProperty('voices')
print len(voices)
# for voice in voices:
print "Using voice:", repr(voices[2])
engine.setProperty('voice', voices[2].id)
engine.say("Hello! Would you like a picture?")
engine.say("Okay, thank you for your time!")

engine.runAndWait()
