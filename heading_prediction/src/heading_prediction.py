#!/usr/bin/env python

import rospy
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32

import cv2
from cv_bridge import CvBridge
import numpy as np

import sys
sys.path.insert(0, '/home/grammers/catkin_ws/src/time_to_collision_calculatin/lib')
from bounding_box import Bounding_box

import copy
import math

sub_calk_data = '/cc/trace'
image_fead = '/bb/image_box'
image_vis = '/hp/image_vis'
gole_topic = '/goal'
heading_topic = '/heading/'
HIGHT = 480
WIDTH = 856

HITBOX = 30
SAFTI = 4

WIDTH_ANGLE = math.radians(80)


class ROS_runner():
    def __init__(self):
        self.claculation_sub = rospy.Subscriber(
            sub_calk_data, Detection2DArray, self.callback)

        self.gole_sub = rospy.Subscriber(
            gole_topic, Float32, self.goal)

        self.heading_pub = rospy.Publisher(
            heading_topic, Float32, queue_size = 1)

        self.image_pub =  rospy.Publisher(
            image_vis, Image, queue_size = 10)

        self.height = 480
        self.width = 856
        self.bridge = CvBridge()
        self.cv_image = np.zeros((self.height, self.width, 3), np.uint8)

        self.goal = 428 
        self.heading = 0
        
    def predict(self, boxes):
        risk = [-1.0 for i in range(self.width)]

        for b in boxes:
            wx, _ = b.get_NW_corner()
            ex, _ = b.get_SE_corner()
            wx *= self.width
            ex *= self.width
            
            for i in range(int(wx), int(ex)):
                if i >= len(risk) or i < 0:
                    continue
                if risk[i] < b.area:
                    risk[i] = b.area
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
            wx, ny = b.get_NW_corner()
            ex, sy = b.get_SE_corner()
            wx, ny = b.point_real(wx, ny)
            ex, sy = b.point_real(ex, sy)
            #print(wx, ny, ex, sy)
            # bbox[x,y, x,y]
            cv2.rectangle(im, (int(wx), int(ny)),
                (int(ex), int(sy)),
                (255, 255, 0), 2)

            cv2.putText(im, str(b.id), (int(wx), int(sy - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 
                int(2))
           
            cv2.arrowedLine(im, (int(b.x), int(b.y)), 
                (int(b.x + b.future_x), int(b.y + b.future_y)),
                (0, 0, 255), int(1), tipLength = 0.5)
            
            #print b.d_area

            cv2.putText(im, str(round(b.area,3)), (int(wx + 2), int(ny - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 
                int(1))
                
        ## hitbox
        cv2.rectangle(im, (int(self.heading - HITBOX), int(self.height / 2)), 
            (int(self.heading + HITBOX), int(HITBOX + self.height / 2)), 
            (0, 255, 0), 2)

        ## GOAL
        cv2.line(im, (self.goal, (self.height / 2) - 5), (self.goal,
            (self.height / 2) + 5), (0, 255, 0), 1)
        cv2.line(im, (self.goal -5, self.height / 2), 
            (self.goal + 5, self.height / 2), (0, 255, 0), 1)

        return im

    def heading_to_angle(self):
        pix = self.heading - self.width / 2
        rad_pix = WIDTH_ANGLE / self.width
        referens = pix * rad_pix
        return referens
        
    def goal(self, data):
        #print("goal")
        #print(data.data)
        
        self.goal = data.data / (WIDTH_ANGLE / self.width)
        #print(self.goal)
        self.goal = int(self.goal + (self.width / 2))
        #print(self.goal)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data.detections[0].source_img, "bgr8")
            self.widht = data.detections[0].source_img.width
            self.height = data.detections[0].source_img.height
        except IndexError:
            print("empty")
        box_list = [] 

        for bbox in data.detections:
            box = Bounding_box(bbox.source_img)
            box.init_fr_msg(bbox)
            box_list.append(box)


        #print(box_list[0].pos_x)
        #print(box_list[0].pos_y)
        
        self.predict(box_list)

    
        self.heading_pub.publish(self.heading_to_angle())
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.visualize(box_list, self.cv_image), "bgr8"))


if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('heading_predicter')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
