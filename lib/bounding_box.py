#!/usr/bin/env python
from collections import deque

from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
#from sensor_msgs.msg import Image

AVG_LEN = 10

class Bounding_box():
    def __init__(self, image):
        self.image = image
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.id = 0 # results.id
        self.area = 0 # results.score
        self.future_x = 0 # results.pose.pose.x
        self.future_y = 0 # results.pose.pose.y

        self.area_history = deque(maxlen=AVG_LEN)
        self.pos_history = deque(maxlen=AVG_LEN)


    def init_fr_msg(self, msg):
        #self.image = msg.source_img    
        
        self.x = msg.bbox.center.x
        self.y = msg.bbox.center.y

        self.w = msg.bbox.size_x
        self.h = msg.bbox.size_y

        self.id = msg.results[0].id

        self.area = msg.results[0].score
        self.future_x = msg.results[0].pose.pose.position.x
        self.future_y = msg.results[0].pose.pose.position.y

    def init_fr_tracker(self, wx, ny, ex, sy, id):
        self.id = id
        #print("")
        #print(wx,ny,ex,sy)
        wx = float(wx) / float(self.image.width)
        ex = float(ex) / float(self.image.width)
        ny = float(ny) / float(self.image.height)
        sy = float(sy) / float(self.image.height)
        #print(wx,ny,ex,sy)

        self.w = ex - wx
        self.h = sy - ny

        self.x = wx + self.w / 2.0
        self.y = ny + self.h / 2.0
        #print(self.x, self.y, self.w, self.h)

    def update(self, bbox):
        self.image = bbox.source_img
        self.x = bbox.bbox.center.x
        self.y = bbox.bbox.center.y
        self.w = bbox.bbox.size_x
        self.h = bbox.bbox.size_y

        self.area_history.append(self.w * self.h)
        self.pos_history.append([self.x, self.y])
    
    def update_area(self):
        self.area = self.area_history[-1] - self.area_history[0]

        x, y = self.get_NW_corner()
        if x < 0.1:
            self.area += 0.5
        if y < 0.1:
            self.area += 0.5

        x, y = self.get_SE_corner()
        if x > 0.9:
            self.area += 0.5
        if y > 0.9:
            self.area += 0.5
        if self.area > 1:
            self.area = 1


    def update_tradj(self):
        self.futrue_x = self.pos_history[-1][0] - self.pos_history[0][0]
        self.future_y = self.pos_history[-1][1] - self.pos_history[0][1]

    def set_bbox(self, bbox_x, bbox_y, bbox_w, bbox_h):
        self.x = bbox_x
        self.y = bbox_y
        self.w = bbox_w
        self.h = bbox_h

    def get_NW_corner(self):
        x = self.x - self.w / 2.0
        y = self.y -self.h / 2.0
        return x, y

    def get_SE_corner(self):
        x = self.x + self.w / 2.0
        y = self.y + self.h / 2.0
        return x, y

    def x_real(self):
        return self.x * self.image.width

    def y_real(self):
        return self.y * self.image.height

    def w_real(self):
        return self.w * self.image.width

    def h_real(self):
        return self.h * self.image.height

    def point_real(self, x, y):
        r_x = x * self.image.width
        r_y = y * self.image.height
        return r_x, r_y
        

    def to_msg(self):
            msg = Detection2D()

            msg.source_img = self.image

            msg.bbox.center.x = self.x
            msg.bbox.center.y = self.y
            msg.bbox.size_x = self.w
            msg.bbox.size_y = self.h
            
            obj = ObjectHypothesisWithPose()
            
            obj.id = self.id
            obj.score = self.area
            obj.pose.pose.position.x = self.future_x
            obj.pose.pose.position.y = self.future_y

            msg.results.append(obj)

            return msg
