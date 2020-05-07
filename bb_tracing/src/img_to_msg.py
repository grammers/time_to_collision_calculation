#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge

import os

img_pub_topic = '/jpg/image'
img_path = '/home/grammers/xjob/indian_dataset/'
start = 4575
stop = 20000

class ROS_runner():
    def __init__(self):
        self.img_pub = rospy.Publisher(
            img_pub_topic, Image, queue_size=10)

        self.bridge = CvBridge()

    def spin(self):
        rate = rospy.Rate(0.45)
       
        for nr in range(start, stop):
            if rospy.is_shutdown():
                break

            file_path = img_path + 'circuit2_x264.mp4 ' + str(nr).zfill(5) + '.jpg'
            print(file_path)
            cv_image = cv2.imread(file_path,3)
            
            #hight, width, channels = cv_image.shape
            #print hight, width, channels
            print nr
            
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

            rate.sleep()

if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('jpg_to_img', anonymous=True)
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
