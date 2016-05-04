'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
Modified by Serena Booth
'''
import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent, Sound
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

import time

#### TEXT TO SPEECH STUFFF ######
import pyttsx
engine = pyttsx.init()
engine.setProperty('rate', 180)

voices = engine.getProperty('voices')
print len(voices)
# for voice in voices:
print "Using voice:", repr(voices[2])
engine.setProperty('voice', voices[2].id)


lin_speed = 0.1
ROT_SPEED = math.radians(30)
bump = False
cliff = False
wheel_drop = False
front_obstacle = False

face = False

# Face detection using the webcam
# Make sure to run webcam.launch
tookPicture = False
turned_back = False
count_down = False
move_on = False

class FaceDetection():
    def processBumpSensing(self, data):
        global bump
        if (data.state == BumperEvent.PRESSED):
            bump = True
        rospy.loginfo("Bump Event")
        rospy.loginfo(data.bumper)

    def processCliffSensing(self, data):
        global cliff
        if (data.state == CliffEvent.CLIFF):
            cliff = True

        rospy.loginfo("Cliff Event")
        rospy.loginfo(data.sensor)

    def processWheelSensing(self, data):
        global wheel_drop
        if (data.state == WheelDropEvent.DROPPED):
            wheel_drop = True

        rospy.loginfo("Wheel Drop Event")
        rospy.loginfo(data.wheel)

     


    def faceDetect(self, image):
        global tookPicture, face, notFaceCentered, count_down, move_on

        faces = self.face_cascade.detectMultiScale(image)


        # for (x, y, w, h) in faces:

        #     if x < 3 or y < 3 or (x + w > 635) or (y + h > 475):
        #         engine.say("Please move closer together! Your face is off the screen")
          
        num_faces = len(faces)

        if num_faces > 0:
         
            face = True

            if not tookPicture and turned_back:
                string = "I detected " + str(num_faces) + " faces"
                # rospy.loginfo(string)
                engine.say(string)
                # engine.runAndWait()

                
                engine.say("three")
                engine.say("two")
                engine.say("one")
                engine.say("stay still now!")
                engine.say("smile!")
                count_down = True
                cv2.imwrite('test.png', image)
                # rospy.loginfo("Picture saved!")
                tookPicture = True
                engine.say("I took your picture!!")
                engine.runAndWait()
                face = False
                move_on = True
        else:
            face = False
            tookPicture = False



        # for (x,y,w,h) in faces:
        #     cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

        #     # add logic here (how many faces, how )
        #     if not tookPicture:
        #         rospy.loginfo("should take pic")
        #         cv2.imwrite('test.png', image)
        #         tookPicture = True
        #     rospy.loginfo("IS THERE A FACE???")
            # self.sound.publish(Sound.ERROR)

        # Display the resulting image
        cv2.imshow('Video', image)
        cv2.waitKey(3)

    def processFaceImage(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.faceDetect(image)

    def processImage(self, data):
        try: 
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            blur_img = cv2.medianBlur(img, 5)

            rows = len(img) #480
            cols = len(img[0]) #640

            cv2.imshow('Cube Detect', img)
            cv2.waitKey(3)

        except CvBridgeError, e: 
            rospy.loginfo(e)

    def processDepthImage(self, data):
        global front_obstacle

        try: 
            img = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            blur_img = cv2.medianBlur(img, 5)

            rows = len(img) #480
            cols = len(img[0]) #640

            # look at top 0.7 and third of each image
            center_h = rows * 0.7
            center_w = cols * 0.3
            cols_half = cols * 0.5

            center_img = blur_img[0:center_h, center_w:(cols-center_w)]
            # left_img = blur_img[0:center_h, 0:(center_w)]
            # right_img = blur_img[0:center_h, (cols-center_w):]
            min_dist = float(np.min(np.nanmin(center_img)))
            # min_left = float(np.min(np.nanmin(left_img)))
            # min_right =  float(np.min(np.nanmin(right_img)))

            if min_dist <= 1.0:
                front_obstacle = True
            # elif min_left <= 1.0:
            #     left_obstacle = True
            # elif min_left <= 1.0:
            #     right_obstacle = True
            else:
                front_obstacle = False
                # left_obstacle = False
                # right_obstacle = False

            # cv2.imshow('Depth Image', depth_array)
            cv2.waitKey(3)

        except CvBridgeError, e: 
            rospy.loginfo(e)

    def __init__(self):
        global bump, cliff, wheel_drop, front_obstacle, face, turned_back, count_down, move_on

        rospy.init_node('FaceDetection', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)


        # print msg 
        rospy.loginfo("Hello World!")
        # How often should provide commands? 10 HZ
        r = rospy.Rate(10);
        self.bridge = CvBridge()

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        rospy.Subscriber('/image_raw', Image, self.processFaceImage)
        # Subscribe to queues for receiving sensory data
        rospy.Subscriber('/camera/depth/image', Image, self.processDepthImage, queue_size=1,  buff_size=2**24)
        # rospy.Subscriber('/camera/rgb/image_raw', Image, self.processImage, queue_size=1,  buff_size=2**24)
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.processBumpSensing)
        rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, self.processWheelSensing)
        rospy.Subscriber('mobile_base/events/cliff', CliffEvent, self.processCliffSensing)
        self.sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        
        

        # Generate a 'left' twist object.
        turn_left = Twist()
        turn_left.linear.x = 0
        turn_left.angular.z = ROT_SPEED #90 deg/s in radians/s
        
        # Twist is a datatype for velocity
        move_cmd = Twist()

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        
        # Subscribe to depth topic 

        while not rospy.is_shutdown(): 
            # calibrate()
            if bump:
                move_cmd.linear.x = 0
                rospy.loginfo("bump")
                for i in range(0,10):
                    move_cmd.linear.x = -lin_speed
                    self.cmd_vel.publish(move_cmd)
                    rospy.loginfo("should move back") 
                    r.sleep()  
                for x in xrange(0,30):
                    turn_left.angular.z = ROT_SPEED
                    self.cmd_vel.publish(turn_left)
                    rospy.loginfo("should turn") 
                    r.sleep()  
                bump = False
            if wheel_drop or cliff:
                rospy.loginfo("cliff or wheel")
                self.shutdown()

            if front_obstacle:
                move_cmd.linear.x = 0
                for x in xrange(0,10):
                    turn_left.angular.z = ROT_SPEED
                    self.cmd_vel.publish(turn_left)
                    rospy.loginfo("should turn") 
                    r.sleep() 

            if face:

                # stop moving when face is detected but photo hasn't been taken
               
                move_cmd.linear.x = 0

                turn_left.angular.z = 0
                self.cmd_vel.publish(turn_left)
                r.sleep() 

                for x in xrange(0,10):
                    turn_left.angular.z = -ROT_SPEED
                    self.cmd_vel.publish(turn_left)
                    r.sleep() 

                # if count_down:
                #     for x in xrange(30):
                #         if x % 10 == 0:
                #             rospy.loginfo(x)
                #             # engine.say('hi')
                #             # engine.runAndWait()
                #         move_cmd.linear.x = 0
                #         self.cmd_vel.publish(move_cmd)
                #         r.sleep() 

                # count_down = False
                turned_back = True      

                # if move_on:
                #     for x in xrange(0,10):
                #         turn_left.angular.z = ROT_SPEED
                #         self.cmd_vel.publish(turn_left)
                #         r.sleep()
                #     move_on = False
            else: 
                turned_back = False  
                move_cmd.linear.x = lin_speed
                self.cmd_vel.publish(move_cmd) 
                # self.sound.publish(Sound.ON)
                r.sleep() 
                # turn_left.angular.z = ROT_SPEED
                # self.cmd_vel.publish(turn_left)
                # r.sleep()   


        
    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Stop")
        rospy.sleep(5)
 
if __name__ == '__main__':
    try:
        FaceDetection()
    except Exception, e:
        print e
        rospy.loginfo("FaceDetection node terminated.")
