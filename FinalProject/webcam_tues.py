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
# import os
# os.system("start C:/thepathyouwant/file")
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
import random

import sys, select 

from play_sound import play_sound
from twitter_streaming import post_twitter

#### TEXT TO SPEECH STUFFF ######
import pyttsx
engine = pyttsx.init()
engine.setProperty('rate', 130)
volume = engine.getProperty('volume')
engine.setProperty('volume', volume+0.25)

voices = engine.getProperty('voices')
print len(voices)
# for voice in voices:
print "Using voice:", repr(voices[2])
engine.setProperty('voice', voices[2].id)


lin_speed = 0.0
ROT_SPEED = math.radians(15)

bump = False
cliff = False
wheel_drop = False
front_obstacle = False

faceDetected = False
notFaceCentered = True 
wantPhoto = False 

# Make sure to run webcam.launch
tookPicture = False
count_down = False
move_on = False

# to increase exploration of photos of different people
photo_timer = -1
num_faces = 0

# PHRASES LIST (Group)
group_phrases = ["Wow, what a photogenic group!", "These pictures are going up on my Twitter"]

# PHRASES LIST (SINGLE)
solo_phrases = ["You look great!", "Where are your friends?", "You are a beauty queen"]

# Self promotion
promotion = ["Make sure to follow me on Twitter at seas underscore photobot!", "Make sure to tell all your friends about PhotoBot", "IMMORTAL POWER"]
# random.choice(foo)


"""""""""""""HELPER FUNCTIONS"""""""""



"""""""""""""Photography Stuff"""""""""

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

    def talk(self):
    	global num_faces 

    	rospy.loginfo("Entering takePicture function!")

        string = "I detected " + str(num_faces) + " faces"
        rospy.loginfo("detected face")

        engine.say("Hello, my name is PhotoBot!")
        engine.say("I would like to take your photo.")
        engine.say("Please take a prop from the basket")
        # engine.say(" Hope you are having fun at the design fair.")
        # engine.say("I'm going to take a picture of you. You are stunning.")
        # engine.say("Stay, still, now!")

        time.sleep(0.2)
        engine.say("three")
        time.sleep(0.2)
        engine.say("two")
        time.sleep(0.2)
        engine.say("one")
        time.sleep(0.2)
        engine.say("Smile!")
        
        time.sleep(0.5)
        
        play_sound()
        # os.system("start C:/camera_shutter.wav")



    def takePicture(self, image):
    	global tookPicture, faceDetected, move_on, photo_timer, num_faces, wantPhoto

        cv2.imwrite('test.png', image)
        rospy.loginfo("Picture saved!")
        post_twitter('test.png')
        time.sleep(1.0)

        if num_faces > 1:
            engine.say(random.choice(group_phrases))
        else:
            engine.say(random.choice(solo_phrases))

        time.sleep(1.0)
        engine.say(random.choice(promotion))

        # insert a sound maybe
        engine.runAndWait()

        tookPicture = True
        faceDetected = False
        wantPhoto = False
        move_on = True
        photo_timer = 5

      

    def faceDetect(self, image):
        global tookPicture, faceDetected, notFaceCentered, count_down, move_on

        # to increase exploration of photos of different people
        global photo_timer, num_faces, wantPhoto

        faces = self.face_cascade.detectMultiScale(image, scaleFactor=1.1, minNeighbors=5, minSize=(25, 25), flags=cv2.cv.CV_HAAR_SCALE_IMAGE)
          
        num_faces = len(faces)


        # when face is detected
        if num_faces > 0:
			rospy.loginfo("Faces are detected")

			faceDetected = True

			if wantPhoto:

				if num_faces == 1:
					rospy.loginfo("One face detected!")

					for (x,y,w,h) in faces:
						if w*h > 30:
							cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 2)

							if x < 50 or y < 50 or (x+w>600) or (y+h>400):
								rospy.loginfo("you are not centered!")
								engine.say("Your face is off the screen")
								engine.runAndWait()
								notFaceCentered = True 

							else: 
								rospy.loginfo("you are centered!")
								notFaceCentered = False
				else:
					rospy.loginfo("Multiple face detected!")

					for (x,y,w,h) in faces:
						if w*h > 30:
							cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 2)

							if x < 50 or y < 50 or (x+w>600) or (y+h>400):
								rospy.loginfo("you are not centered!")
								engine.say("Your face is off the screen")
								engine.runAndWait()
								notFaceCentered = True 

							else: 
								rospy.loginfo("you are centered!")
								notFaceCentered = False
		   
			else:
				engine.say("Please press enter if you'd like a photo")
				engine.runAndWait()

				rospy.loginfo("Please press ENTER for a photo")

				i, o, e = select.select([sys.stdin], [], [], 10)
				if (i):
					rospy.loginfo("yay photo time!")
					wantPhoto = True
				else:
					rospy.loginfo("aw photo next time")
					wantPhoto = False 
	
        else:
        	# if there is no face detected 
            faceDetected = False
            tookPicture = False

        cv2.imshow('Livestream', image)
        cv2.waitKey(5)
        



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
        

    def processFaceImage(self, data):
    	# to increase exploration of photos of different people
        global photo_timer, notFaceCentered 

        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # grab the new image when photo_time = -1
        if photo_timer < 0:

        	image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       
        	if notFaceCentered:
        		self.faceDetect(image)
        	else:
        		self.talk()
        		self.takePicture(image)
        else:
        	pass 
        		
 

    def processImage(self, data):
        try: 
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            blur_img = cv2.medianBlur(img, 5)

            rows = len(img) #480
            cols = len(img[0]) #640
            cv2.imshow('Cube Detect', img)
            # cv2.waitKey(3)

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
        global bump, cliff, wheel_drop, front_obstacle, faceDetected, count_down, move_on

        # to increase exploration of photos of different people
        global photo_timer, wantPhoto

          

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
        rospy.Subscriber('/image_raw', Image, self.processFaceImage, queue_size=1, buff_size=2**24)
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
            # if bump:
            #     move_cmd.linear.x = 0
            #     rospy.loginfo("bump")
            #     for i in range(0,10):
            #         move_cmd.linear.x = -lin_speed
            #         self.cmd_vel.publish(move_cmd)
            #         rospy.loginfo("should move back") 
            #         r.sleep()  
            #     for x in xrange(0,30):
            #         turn_left.angular.z = ROT_SPEED
            #         self.cmd_vel.publish(turn_left)
            #         rospy.loginfo("should turn") 
            #         r.sleep()  
            #     bump = False

            # if wheel_drop or cliff:
            #     rospy.loginfo("cliff or wheel")
            #     self.shutdown()

            # if front_obstacle:
            #     move_cmd.linear.x = 0
            #     for x in xrange(0,10):
            #         turn_left.angular.z = ROT_SPEED
            #         self.cmd_vel.publish(turn_left)
            #         # rospy.loginfo("Obstacle in front") 
            #         r.sleep() 

            # pausing in between taking photos 
      		if photo_timer > -1:
      			rospy.loginfo("waiting period before next pic" + str(photo_timer))
      			photo_timer -= 1

            if faceDetected or wantPhoto:
                # stop moving when face is detected but photo hasn't been taken
                # move_cmd.linear.x = 0
                # self.cmd_vel.publish(move_cmd) 

                turn_left.angular.z = 0 
                self.cmd_vel.publish(turn_left)
                r.sleep()

                if move_on:
                	rospy.loginfo("moving on now")
                	for x in xrange(0,10): 
                		rospy.loginfo("turning")
                		turn_left.angular.z = ROT_SPEED
                		self.cmd_vel.publish(turn_left)
                		r.sleep()
                	move_on = False
                	faceDetected = False
                	wantPhoto = False 
               
            else: 
            	# navigate the room normally otherwise
            
                # move_cmd.linear.x = lin_speed
                # self.cmd_vel.publish(move_cmd) 
                # # self.sound.publish(Sound.ON)
                # r.sleep() 
                rospy.loginfo("should be turning in normal state")
                turn_left.angular.z = ROT_SPEED
                self.cmd_vel.publish(turn_left)
                r.sleep()   


        
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
