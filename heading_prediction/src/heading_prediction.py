#!/usr/bin/env python

import rospy
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

sub_calk_data = '/bbox_avoid/calc'
image_vis = '/bbox_avoid/viz'
angle_to_waypoint_topic = '/bbox_avoid/angle_waypoint'
heading_topic = '/bbox_avoid/heading'
HIGHT = 480
WIDTH = 856

# estimated collision size in one direction
HITBOX = 30
# nr times one size that should be safe zone
SAFETY = 4

WIDTH_ANGLE = math.radians(80)


class ROS_runner():
    def __init__(self):
        self.calculation_sub = rospy.Subscriber(
            sub_calk_data, Detection2DArray, self.callback)

        self.angle_waypoint_sub = rospy.Subscriber(
            angle_to_waypoint_topic, Float32, self.waypoint)

        self.heading_pub = rospy.Publisher(
            heading_topic, Float32, queue_size = 1)

        self.image_pub =  rospy.Publisher(
            image_vis, Image, queue_size = 10)

        self.height = 480
        self.width = 856
        self.bridge = CvBridge()
        self.cv_image = np.zeros((self.height, self.width, 3), np.uint8)

        self.waypoint = 428 
        self.heading = 0
        
    def predict(self, boxes):
        # generate a array withe collision risks
        # -1 is max safety should probably be some thing ells.
        # obstacles moving away (<0) is a safe to follow
        # set to -1 to simplify testing
        risk = [-1.0 for i in range(self.width)]

        # loop thru all boxes
        for b in boxes:
            # extract corners and denormalise them
            wx, _ = b.get_NW_corner()
            ex, _ = b.get_SE_corner()
            wx *= self.width
            ex *= self.width
            
            # up date collision risks
            # for pixels in bounding boxes are the risk = area growth
            # the highest risk for each pixel is used.
            for i in range(int(wx), int(ex)):
                if i >= len(risk) or i < 0:
                    continue
                if risk[i] < b.area:
                    risk[i] = b.area
        # find the safest heading
        self.find_heading(risk)
        #print(risk)

    # calculate the upper and lover boundaries for a sup set
    def get_bounds(self, mid, roof, offset):
        lover = mid - HITBOX * SAFETY + offset
        higher = mid + HITBOX * SAFETY + offset
        
        # force to stick inside the image frame
        if lover < 0:
            higher += -lover
            lover = 0
        if higher >= roof:
            lover -= higher - roof
            higher = roof - 1

        return lover, higher
        
    # input risk array
    def find_heading(self, risk):
        # start search from waypoint
        mid = self.waypoint #len(risk) / 2
        # minimum initialized ass theoretical max dangers.
        # multiplied by 2 to consider left and right side
        minimum = 1 * 2 * HITBOX * SAFETY
        # start width heading state to waypoint
        self.heading = self.waypoint

        # ind sub array of risk that has the lowest minimum
        # lop thru all possible pixels is side fov
        for offset in range(int(mid - HITBOX * SAFETY)):
            # start with offset to the right
            lover, higher = self.get_bounds(mid, len(risk), offset)
            current = self.sum_span(risk[lover : higher])
            # if current heading is safer store it ass a new heading
            if minimum > current:
                minimum = current
                self.heading = mid + offset
            # check for offsets to the left
            lover, higher = self.get_bounds(mid, len(risk), - offset)
            current = self.sum_span(risk[lover : higher])
            if minimum > current:
                minimum = current
                self.heading = mid - offset
        

    # tack a sub array
    # returns the sum of the values in that array
    def sum_span(self, array):
        sum = 0
        for s in array:
            sum += s
        return sum

    # draw stuff on image for visualisation
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
        cv2.line(im, (self.waypoint, (self.height / 2) - 5), (self.waypoint,
            (self.height / 2) + 5), (0, 255, 0), 1)
        cv2.line(im, (self.waypoint -5, self.height / 2), 
            (self.waypoint + 5, self.height / 2), (0, 255, 0), 1)

        return im

        # convert pixel heading to angel
    def heading_to_angle(self):
        # offset sow center pixels is 0
        pixel_heading = self.heading - self.width / 2
        # nr radians per pixel
        radian_pixel = WIDTH_ANGLE / self.width
        # nr pixels of from center time radians per pixel give a angel in radians
        angle_to_waypoint = pixel_heading * radian_pixel
        return angle_to_waypoint
    
    # get angel to next waypoint
    # convert angle to pixels
    def waypoint(self, data):
        # angel to next waypoint / angel per pixel
        # returns the numbers of pixels that corresponds to the angle
        self.waypoint = data.data / (WIDTH_ANGLE / self.width)
        # angle 0 is start ahead
        # offset sow mid off image are strait ahead
        self.waypoint = int(self.waypoint + (self.width / 2))

    ## callback for new frames with bounding boxes
    def callback(self, data):
        try:
            # read data about new image
            self.cv_image = self.bridge.imgmsg_to_cv2(data.detections[0].source_img, "bgr8")
            self.widht = data.detections[0].source_img.width
            self.height = data.detections[0].source_img.height
        except IndexError:
            print("empty")
        box_list = [] 

        # extract bounding boxes form messages
        for bbox in data.detections:
            box = Bounding_box(bbox.source_img)
            box.init_fr_msg(bbox)
            box_list.append(box)

        
        # calculate a suggested heading
        self.predict(box_list)

    
        self.heading_pub.publish(self.heading_to_angle())
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.visualize(box_list, self.cv_image), "bgr8"))


if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('heading_predictor')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
