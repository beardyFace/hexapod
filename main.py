#!/usr/bin/python
from time import sleep
from hexapod import Hexapod
import sys

import rospy
import roslib
import cv2
import numpy as np

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import queue

class App():
    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joyCallback)
        
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("hex_image",Image,queue_size=10)
        self.cam = cv2.VideoCapture(0)

        self.command_queue = queue.Queue(maxsize=1)

        self.hexapod = Hexapod()

    def joyCallback(self, joy_command):
        self.command_queue.put(joy_command)

    def moveHexapod(self, joy_command):
        axes    = joy_command.axes
        buttons = joy_command.buttons

        linear_d = 30
        dx  = 0#axes[0] * linear_d
        dy  = 0#axes[1] * linear_d
        dz  = 0#axes[2] * linear_d

        rot_d = 30
        rx = 0#axes[3] * rot_d
        ry = 0#axes[4] * rot_d
        rz = 0#axes[5] * rot_d

        self.hexapod.setBody(dx,dy,dz,rx,ry,rz)

    def run(self):
        cv2.namedWindow('webcam', cv2.WINDOW_NORMAL)
        cv2.moveWindow('webcam', 100,100)
        cv2.resizeWindow('webcam', 500, 500)

        rate = rospy.Rate(30)#Hz
        dx = 0
        dy = 0
        dz = 0
        d = 0.1
        while not rospy.is_shutdown():
            try:
                ret_val, cv_image = self.cam.read()
                if ret_val:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                    cv2.imshow('webcam', cv_image)
                    cv2.waitKey(1)

                try:
                    joy_command = self.command_queue.get_nowait()
                    self.moveHexapod(joy_command)
                except queue.Empty:
                    pass#no command in the queque

                self.hexapod.setBody(dx,dy,dz,0,0,5)
                dx += d
                dy += d
                dz += d
                if dx > 30:
                    d = -d
                elif d < -30:
                    d = -d

            except CvBridgeError as e:
                print(e)
            rate.sleep()

def main(argv):
    rospy.init_node('hexapod_node', anonymous=False)

    app = App()
    app.run()
    
if __name__ == "__main__":
    main(sys.argv)
