import speech_recognition as sr
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import time

class Listen():

	def callback(recognizer, audio):                          # this is called from the background thread
    try:
        print("You said " + recognizer.recognize(audio))  # received audio data, now need to recognize it
    except LookupError:
        print("Oops! Didn't catch that")

    def __init__(self):
        # initiliaze
        rospy.init_node('turnleft', anonymous=False)

        # What to do you ctrl + c (call shutdown function written below)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
        	r = sr.Recognizer()
        	m = sr.Microphone()

        	with m as source:
        		r.adjust_for_ambient_noise(source)
        	stop_listening = r.listen_in_background(m, callback)

        	for _ in range(50):
        		time.sleep(0.1)
        	stop_listening()

        	while True:
        		time.sleep(0.1)
           
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop!")
        # publish a zeroed out Twist object
        self.cmd_vel.publish(Twist())
        # sleep before final shutdown
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Listen()
    except:
        rospy.loginfo("node terminated.")





