import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class TurnLeft():
    def __init__(self):
        # initiliaze
        rospy.init_node('turnleft', anonymous=False)

        # What to do you ctrl + c (call shutdown function written below)
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.sleep(1) # give rospy time to register the publisher
         
        soundhandle = SoundClient()
        rospy.sleep(2)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            soundhandle.say("Hello world!")
            rospy.sleep(3)
        
            
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop!")
        # publish a zeroed out Twist object
        self.cmd_vel.publish(Twist())
        # sleep before final shutdown
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        TurnLeft()
    except:
        rospy.loginfo("node terminated.")


