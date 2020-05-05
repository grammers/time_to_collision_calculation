#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

from collections import deque
import math

sub_bbox_topic = '/bb/trace'
# camera information
HIGHT = 480.0
WITH = 856.0
MID = HIGHT / 2.0
HIGHT_ANGLE = 39.3

AVG_LEN = 10

PIXEL_ANEL = math.radians(HIGHT / HIGHT_ANGLE)

class Traces():
    def __init__(self, ID, dist, frame):
        self.id = ID
        self.distance = deque(maxlen=AVG_LEN)
        self.distance.append(dist)
        self.speed = 1.0
        self.time = 0.0

        self.area = deque(maxlen=AVG_LEN)
        self.d_area = 0.0

        self.center = deque(maxlen=AVG_LEN)

        self.frame_of_update = frame

    def avg_dist(self):
        dist = 0.0
        for d in self.distance:
            dist += d
        return dist / AVG_LEN
    
    def update_center(self, pos, size):
        self.center.append([
            (pos.x + size.x / 2) - (WITH / 2), 
            (pos.y + size.y / 2) - (HIGHT / 2)])
        

    def area_update(self, a):
        self.area.append(a)
        self.d_area = (self.area[-1] - self.area[0]) / len(self.area)

    def update(self, dist, frame):
        self.distance.append(dist)
        self.speed = self.distance[-1] - self.distance[0]
        if self.speed != 0:
            self.time = self.avg_dist() / self.speed

        self.frame_of_update = frame

    def is_active(self, curent):
        return self.frame_of_update < curent - 1

    def get_dist(self):
        return self.distance[-1]

    def get_speed(self):
        return self.speed

    def get_time(self):
        return self.time

    def __str__(self):
        direction = [self.center[0][0] - self.center[-1][0], self.center[0][1] - self.center[-1][1]]
        return "ID: " + str(self.id) + "\t speed: " + str(self.time) + "\t time: " + str(self.speed) + "\t area: " + str(self.d_area) + "\t direction: " + str(direction)

class ROS_runner():
    def __init__(self):
        self.bbox_sub = rospy.Subscriber(
            sub_bbox_topic, BoundingBoxArray, self.callback)
        
        self.bboxes = {}

        self.distance = {}
        self.speed = {}
        self.time = {}

    def distance_calk(self, box):
        if box.pose.position.y + box.dimensions.y < MID:
            hight1 = MID - box.pose.position.y
            hight2 = MID - box.pose.position.y + box.dimensions.y

        elif box.pose.position.y > MID:
            hight1 = box.pose.position.y - MID
            hight2 = box.pose.position.y + box.dimensions.y - MID
        
        else:
            hight1 = MID - box.pose.position.y
            hight2 = box.pose.position.y + box.dimensions.y - MID
            

        d1 = hight1 / math.atan(hight1 * HIGHT_ANGLE)
        d2 = hight2 / math.atan(hight2 * HIGHT_ANGLE)
        return (d1 + d2) / 2

    def callback(self, data):
        print('callback')
        for box in data.boxes:
            new_distance = self.distance_calk(box)
            
            if box.label in self.bboxes:
                self.bboxes[box.label].update(new_distance, data.header.seq)
            else:
                self.bboxes.update({box.label : Traces(box.label, new_distance, data.header.seq)})


            self.bboxes[box.label].area_update(box.dimensions.y * box.dimensions.x)

            self.bboxes[box.label].update_center(box.pose.position, box.dimensions)

        to_del = []
        for b in self.bboxes:
            print(self.bboxes[b])
            if self.bboxes[b].is_active(data.header.seq):
                to_del.append(b)
            
        for b in to_del:
            self.bboxes.pop(b)
        
        
if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('collision_calculator')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

