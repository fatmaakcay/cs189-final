import os 
import random 
import time
from play_sound import play_sound

phrases = ["They dont want you to win, Bless up, Lie auhn"]


os.system("say " + random.choice(phrases))

play_sound('sounds/played.wav')

# time.sleep(15.0)
print "done??"