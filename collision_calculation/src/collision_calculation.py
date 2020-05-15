#!/usr/bin/env python

import rospy
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from vision_msgs.msg import Detection2DArray

import sys
sys.path.insert(0, '/home/grammers/catkin_ws/src/time_to_collision_calculatin/lib')
from bounding_box import Bounding_box

import math

sub_bbox_topic = '/bb/trace'
pub_extracted_data = '/cc/trace'
## camera information
#bebop
#HIGHT = 480.0
#WITH = 856.0
#HIGHT_ANGLE = 39.3
#indian fead
HIGHT = 272.0
WITH = 480.0
HIGHT_ANGLE = 39.3

MID = HIGHT / 2.0

AVG_LEN = 10

PIXEL_ANEL = math.radians(HIGHT / HIGHT_ANGLE)

class ROS_runner():
    def __init__(self):
        self.bbox_sub = rospy.Subscriber(
            sub_bbox_topic, Detection2DArray, self.callback)
        
        self.extracted_data_pub = rospy.Publisher(
            pub_extracted_data, Detection2DArray, queue_size=10)

        self.bboxes = {}

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

    def msg_builder(self, header):
        msg = BoundingBoxArray()
        msg.header =  header

        for b in self.bboxes:
            msg.boxes.append(self.bboxes[b].to_msg())
        return msg

    def callback(self, data):
        active_id = []
        #print('callback')
        for bbox in data.detections:
            active_id.append(bbox.results[0].id)
            if bbox.results[0].id in self.bboxes:
                self.bboxes[bbox.results[0].id].update(bbox)
            else:
                box = Bounding_box(bbox.source_img)
                box.init_fr_msg(bbox)
                box.update(bbox)
                self.bboxes.update({bbox.results[0].id : box})
            self.bboxes[bbox.results[0].id].update_area()
            self.bboxes[bbox.results[0].id].update_tradj()


        
        msg = Detection2DArray()
        msg.header = data.header
        for index in active_id:
            msg.detections.append(self.bboxes[index].to_msg())
        self.extracted_data_pub.publish(msg)
        
        
if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('collision_calculator')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

