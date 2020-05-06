#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

sub_calk_data = '/cc/trace'
image_fead = '/bb/image_box'
image_vis = '/hp/image_vis'
HIGHT = 480
WIDTH = 856

class Trace():
    def __init__(self, box):
        self.ID = box.label

        self.d_area = box.value

        self.pos_x = box.pose.position.x
        self.pos_y = box.pose.position.y
        
        self.heading_x = box.pose.orientation.x
        self.heading_y = box.pose.orientation.y

        self.withe = box.dimensions.x
        self.hight = box.dimensions.y
        
        self.x1 = self.pos_x - (self.withe / 2)
        self.y1 = self.pos_y - (self.hight / 2)
        self.x2 = self.pos_x + (self.withe / 2)
        self.y2 = self.pos_y + (self.hight / 2)


class ROS_runner():
    def __init__(self):
        self.claculation_sub = rospy.Subscriber(
            sub_calk_data, BoundingBoxArray, self.callback)

        self.img_sub = rospy.Subscriber(
            image_fead, Image, self.image)

        self.image_pub =  rospy.Publisher(
            image_vis, Image, queue_size = 10)

        self.bridge = CvBridge()
        self.cv_image = np.zeros((HIGHT, WIDTH, 3), np.uint8)


    def visualize(self, boxes, im):
        for b in boxes:
           
            cv2.arrowedLine(im, (int(b.pos_x), int(b.pos_y)), 
                (int(b.pos_x + b.heading_x), int(b.pos_y + b.heading_y)),
                (0, 0, 255), int(1), tipLength = 0.5)
            

            cv2.putText(im, str(int(b.d_area)), (int(b.x1), int(b.y1 - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, int(1), (0, 0, 255), 
                int(2))
        return im

    def image(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

    def callback(self, data):
        box_list = [] 

        for box in data.boxes:
            box_list.append(Trace(box))

        print(box_list[0].pos_x)
        print(box_list[0].pos_y)


        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.visualize(box_list, self.cv_image), "bgr8"))


if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('heading_predicter')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
