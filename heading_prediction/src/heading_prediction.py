#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import cv2
from cv_bridge import CvBridge
import numpy as np

import copy

sub_calk_data = '/cc/trace'
image_fead = '/bb/image_box'
image_vis = '/hp/image_vis'
gole_topic = '/goal'
heading_topic = '/heading/'
HIGHT = 480
WIDTH = 856

HITBOX = 30
SAFTI = 3

class Trace():
    def __init__(self, box, width, hight):
        #print width, hight
        self.ID = box.label

        self.d_area = box.value

        self.pos_x = box.pose.position.x 
        self.pos_y = box.pose.position.y 
        
        self.heading_x = box.pose.orientation.x * width
        self.heading_y = box.pose.orientation.y * hight

        #print box
        self.withe = box.dimensions.x 
        self.hight = box.dimensions.y
        
        #print(self.withe)
        self.x1 = self.pos_x - (self.withe / 2)
        self.y1 = self.pos_y - (self.hight / 2)
        self.x2 = self.pos_x + (self.withe / 2)
        self.y2 = self.pos_y + (self.hight / 2)

        #print self.x1
        #print self.y1

        self.pos_x *= width
        self.pos_y *= hight
        self.withe *= width
        self.hight *= hight
        self.x1 *= width
        self.y1 *= hight
        self.x2 *= width
        self.y2 *= hight


class ROS_runner():
    def __init__(self):
        self.claculation_sub = rospy.Subscriber(
            sub_calk_data, BoundingBoxArray, self.callback)

        self.img_sub = rospy.Subscriber(
            image_fead, Image, self.image)

        self.gole_sub = rospy.Subscriber(
            gole_topic, Int32, self.goal)

        self.heading_pub = rospy.Publisher(
            heading_topic, Int32, queue_size = 1)

        self.image_pub =  rospy.Publisher(
            image_vis, Image, queue_size = 10)

        self.hight = 480
        self.width = 856
        self.bridge = CvBridge()
        self.cv_image = np.zeros((self.hight, self.width, 3), np.uint8)

        self.goal = 428 
        self.heading = 0
        
    def predict(self, boxes):
        risk = [0.0 for i in range(self.width)]

        for b in boxes:
            for i in range(int(b.x1), int(b.x2)):
                if risk[i] < b.d_area:
                    risk[i] = b.d_area
        self.find_heading(risk)
        #print(risk)

    def get_bounds(self, mid, rof, offset):
        lover = mid - HITBOX * SAFTI + offset
        higer = mid + HITBOX * SAFTI + offset
        
        if lover < 0:
            higer += -lover
            lover = 0
        if higer >= rof:
            lover -= higer - rof
            higer = rof - 1

        return lover, higer
        
    def find_heading(self, risk):
        mid = self.goal #len(risk) / 2
        minimum = 1 * 2 * HITBOX * SAFTI
        self.heading = self.goal

        for offset in range(int(mid - HITBOX * SAFTI)):
            lover, higer = self.get_bounds(mid, len(risk), offset)
            current = self.sum_span(risk[lover : higer])
            if minimum > current:
                minimum = current
                self.heading = mid + offset
            lover, higer = self.get_bounds(mid, len(risk), - offset)
            current = self.sum_span(risk[lover : higer])
            if minimum > current:
                minimum = current
                self.heading = mid - offset
        

    def sum_span(self, array):
        summa = 0
        for s in array:
            summa += s
        return summa

    def visualize(self, boxes, im):
        for b in boxes:
            # bbox[x,y, x,y]
            cv2.rectangle(im, (int(b.x1), int(b.y1)),
                (int(b.x2), int(b.y2)),
                (255, 255, 0), 2)

            cv2.putText(im, str(b.ID), (int(b.x1), int(b.y2 - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 
                int(2))
           
            cv2.arrowedLine(im, (int(b.pos_x), int(b.pos_y)), 
                (int(b.pos_x + b.heading_x), int(b.pos_y + b.heading_y)),
                (0, 0, 255), int(1), tipLength = 0.5)
            
            #print b.d_area

            cv2.putText(im, str(round(b.d_area,3)), (int(b.x1), int(b.y1 - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 
                int(1))
                
        ## hitbox
        cv2.rectangle(im, (int(self.heading - HITBOX), int(self.hight / 2)), 
            (int(self.heading + HITBOX), int(HITBOX + self.hight / 2)), 
            (0, 255, 0), 2)

        return im

    def goal(self, data):
        self.goal = data.data

    def image(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        self.hight, self.width, channesl = self.cv_image.shape

    def callback(self, data):
        box_list = [] 

        for box in data.boxes:
            box_list.append(Trace(box, self.width, self.hight))

        #print(box_list[0].pos_x)
        #print(box_list[0].pos_y)
        
        self.predict(box_list)

        
        self.heading_pub.publish(self.heading - self.withe / 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.visualize(box_list, copy.copy(self.cv_image)), "bgr8"))


if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('heading_predicter')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
