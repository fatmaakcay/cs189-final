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
import random

import sys, select 
import os 

from play_sound import play_sound
from twitter_win import post_twitter

lin_speed = 0.00 #m/s
ROT_SPEED = math.radians(0)
turn_d = 1
bump = False
obstacle = False 
left_obstacle = False
right_obstacle = False
cliff = False
wheel_drop = False

faceDetected = False
notFaceCentered = True 
wantPhoto = False 
introduced = False
introduce_yourself = False

tookPicture = False
move_on = False
count_down = 3
g_img = None

num_faces = 0

# PHRASES LIST (Group)
phrases = ["They dont want you to win", "The key to success is good pictures", "Bless up", "Lie auhn", "They never said winning was easy"]

# Self promotion
promotion = ["We da best music", "The key to success is to follow me on Twitter", "The key to success is IMMORTAL POWER", "Some people cant handle winning, I can."]
# random.choice(foo)

sound_files = ["sounds/inspire.wav", "sounds/jewlery.wav", "sounds/smart.wav"]

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

    def face_detect(self, image):
        global faceDetected, num_faces

        detected = False

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(image, scaleFactor=1.1, minNeighbors=5, minSize=(25, 25), flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        # add logic here to make sure not every face is detected 
        num_faces = len(faces) 
        if num_faces > 0:
            faceDetected = True
        else:
            faceDetected = False

        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow('Video',image)
        cv2.waitKey(5)


    def waitforInput(self, timeout):
        i, o, e = select.select([sys.stdin], [], [], timeout)
        
        if (i):
            inputString = sys.stdin.readline().strip()
            return True

        return False

    def introduceMyself(self):
        global wantPhoto, introduced

        rospy.loginfo("should introduce")

        wp = False
        itr = False

        if not introduced:
            os.system("say 'Press enter to ride with me through the journey of more success.'")
            
            timeout = 10.0

            i = self.waitforInput(timeout)

            if i:
                rospy.loginfo(str(i))
                os.system("say ''")
                os.system("say 'The key to success is great props, take a prop from the basket.'")
                # time.sleep(10.0)
                os.system("say 'Are you ready?'")
                os.system("say 'three, two, one, bless up'")
                wantPhoto = True
            else:
                rospy.loginfo(str(i))
                os.system("say 'The key is to press enter.")
                play_sound('sounds/played.wav')
                wantPhoto = False

        introduced = True
        return (wantPhoto, introduced)
        # i, o, e = select.select([sys.stdin], [], [], 1.0)
        
    def takePhoto(self, image):
    	global tookPicture, move_on, num_faces, wantPhoto, introduced, faceDetected, introduce_yourself, count_down, g_img

        image = self.bridge.imgmsg_to_cv2(image, "bgr8")

        g_img = image

        self.face_detect(image)

        if faceDetected and not introduced:
            introduce_yourself = True
            rospy.loginfo("should set introduce_yourself to True")

    def takePicture(self, image):
        global tookPicture, wantPhoto, introduced, faceDetected, count_down, g_img

        if faceDetected and introduced and wantPhoto and not tookPicture:
            # take_three = bool(random.getrandbits(1))

            # if take_three:
            #     rospy.loginfo("gonna take 3 pics")
            #     for x in xrange(count_down):
            #         if x == 2:
            #             cv2.imwrite("image" + str(x+1) + ".png", l_img)
            #             play_sound('sounds/camera_shutter.wav')
            #             post_twitter(take_three)
            #             rospy.loginfo("Image posted to twitter")
            #             os.system("say " + random.choice(phrases))
            #             os.system("say " + random.choice(promotion))
            #             tookPicture = True
            #         else:
            #             l_img = g_img
            #             x_offset=120
            #             y_offset=175

            #             # import in the harvard logo
            #             s_img = cv2.imread("SEASLogo1.png", -1)

            #             for c in range(0,3):
            #                 l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] = s_img[:,:,c] * (s_img[:,:,3]/255.0) +  l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] * (1.0 - s_img[:,:,3]/255.0)
                        
            #             cv2.imwrite("image" + str(x+1) + ".png", l_img)
            #             play_sound('sounds/camera_shutter.wav')
            #             play_sound('sounds/anotherone.wav')
            #             # time.sleep(2.0)
            # else:
                l_img = g_img
                x_offset=120
                y_offset=175

                # import in the harvard logo
                s_img = cv2.imread("SEASLogo1.png", -1)

                for c in range(0,3):
                    l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] = s_img[:,:,c] * (s_img[:,:,3]/255.0) +  l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] * (1.0 - s_img[:,:,3]/255.0)

                cv2.imwrite("new_photo.png", l_img)
                play_sound('sounds/camera_shutter.wav')
                post_twitter(take_three)
                rospy.loginfo("Image posted to twitter")

                os.system("say " + random.choice(phrases))
                os.system("say " + random.choice(promotion))

        tookPicture = True
        return tookPicture

    # obstacle avoidance from last pset
    def processDepthImage(self, data):
        global obstacle, left_obstacle, right_obstacle
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
            left_img = blur_img[0:center_h, 0:(center_w)]
            right_img = blur_img[0:center_h, (cols-center_w):]
            min_dist = float(np.min(np.nanmin(center_img)))
            min_left = float(np.min(np.nanmin(left_img)))
            min_right =  float(np.min(np.nanmin(right_img)))

            if min_dist <= 1.0:
                obstacle = True
            elif min_left <= 1.0:
                left_obstacle = True
            elif min_left <= 1.0:
                right_obstacle = True
            else:
                obstacle = False
                left_obstacle = False
                right_obstacle = False

        except CvBridgeError, e: 
            rospy.loginfo(e)

    def __init__(self):
        global bump, cliff, wheel_drop, faceDetected, move_on, obstacle, introduced, wantPhoto, introduce_yourself, tookPicture, g_img
          

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
        rospy.Subscriber('/image_raw', Image, self.takePhoto, queue_size=1, buff_size=2**24)

        # Subscribe to queues for receiving sensory data
        rospy.Subscriber('/camera/depth/image', Image, self.processDepthImage, queue_size=1,  buff_size=2**24)
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

        while not rospy.is_shutdown(): 
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
                move_cmd.angular.z = 0
                move_cmd.linear.x = 0
                rospy.loginfo("cliff or wheel")
                rospy.sleep(10)

            if faceDetected or (faceDetected and obstacle):
                rospy.loginfo("DETECTED FACE")
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0
                self.cmd_vel.publish(move_cmd) 
                r.sleep()

                if introduce_yourself:
                    rospy.loginfo("should be introducing")
                    wantPhoto, introduced = self.introduceMyself()

                if not wantPhoto and introduced:
                    move_on = True

                if wantPhoto:
                    rospy.loginfo("taking photo in if face detected")
                    tookPicture = self.takePicture(g_img)

                if tookPicture:
                    os.system("say 'The key to success is to take photos of more people, bye.'")
                    rospy.loginfo("moving on now")
                    for x in xrange(0,30): 
                        rospy.loginfo("turning in face detected")
                        move_cmd.angular.z = math.radians(0)
                        self.cmd_vel.publish(move_cmd) 
                        r.sleep()
                if move_on:
                    os.system("say 'The key to success is to find people to take photos of.'")
                    for x in xrange(0,30): 
                        rospy.loginfo("moving on")
                        move_cmd.angular.z = math.radians(0)
                        self.cmd_vel.publish(move_cmd) 
                        r.sleep()

                move_on = False
                wantPhoto = False
                tookPicture = False
                introduced = False
                introduce_yourself = False
                # faceDetected = False
            elif obstacle:
                rospy.loginfo("obstacle")
                # while obstacle: 
                #     move_cmd.linear.x = 0
                #     if not left_obstacle:
                #         turn_left.angular.z = math.radians(45) 
                #         self.cmd_vel.publish(turn_left)
                #         rospy.loginfo("obstacle, turn left")
                #         r.sleep()
                #     elif not right_obstacle:
                #         turn_left.angular.z = -math.radians(45)
                #         self.cmd_vel.publish(turn_left)
                #         rospy.loginfo("obstacle, turn right")
                #         r.sleep()
                #     else:
                #         turn_left.angular.z = math.radians(45)
                #         self.cmd_vel.publish(turn_left)
                #         rospy.loginfo("detected on both sides")
                #         r.sleep()     
            else: 
            	# navigate the room normally otherwise
                rospy.loginfo("moving forward bc no obstacle")
                move_cmd.linear.x = lin_speed
                move_cmd.angular.z = 0
                self.cmd_vel.publish(move_cmd) 
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