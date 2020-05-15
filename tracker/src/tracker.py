#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""@author: kyleguan
"""

import sys
sys.path.insert(0, '/home/grammers/catkin_ws/src/time_to_collision_calculatin/lib')
from bounding_box import Bounding_box

import rospy
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from vision_msgs.msg import Detection2DArray
import numpy as np
#import matplotlib.pyplot as plt
#import glob
#from moviepy.editor import VideoFileClip
from collections import deque
from sklearn.utils.linear_assignment_ import linear_assignment

#import helpers
#import detector
import kalman_tracker

pub_trace = "/bb/trace"
sub_boxes = "/yolo/bb_arr"

class ROS_runner():
    def __init__(self):
        # Global variables to be used by funcitons of VideoFileClop
        self.frame_count = 0 # frame counter

        self.max_age = 4  # no.of consecutive unmatched detection before 
                     # a track is deleted

        self.min_hits =1  # no. of consecutive matches needed to establish a track

        self.tracker_list =[] # list for trackers
        self.id = 0

        #debug = True

        self.boxes_sub = rospy.Subscriber(
            sub_boxes, Detection2DArray, self.callback)
        self.trace_pub = rospy.Publisher(
            pub_trace, Detection2DArray, queue_size = 10)

    def callback(self, data):
        boxes = []
        for trk in data.detections:
            bbox = Bounding_box(trk.source_img)
            bbox.init_fr_msg(trk)
            
            wx, ny = bbox.get_NW_corner()
            wx, ny = bbox.point_real(wx, ny)
            ex, sy = bbox.get_SE_corner()
            ex, sy = bbox.point_real(ex, sy)
            boxes.append([ny, wx, sy, ex])

        self.pipeline(boxes)
        
        msg = Detection2DArray()
        msg.header = data.header
        for trk in self.tracker_list:
            try:
                #print(trk.box[1])
                bbox = Bounding_box(data.detections[0].source_img)
                #print(data.detections[0].source_img.width)
                bbox.init_fr_tracker(trk.box[1], trk.box[0], trk.box[3], trk.box[2], trk.id)
                msg.detections.append(bbox.to_msg())
                #print(bbox.x)
            except IndexError:
                print("empty")
        self.trace_pub.publish(msg)

    def assign_detections_to_trackers(self, trackers, detections, iou_thrd = 0.3):
        '''
        From current list of trackers and new detections, output matched detections,
        unmatchted trackers, unmatched detections.
        '''    
        
        IOU_mat= np.zeros((len(trackers),len(detections)),dtype=np.float32)
        for t,trk in enumerate(trackers):
            #trk = convert_to_cv2bbox(trk) 
            for d,det in enumerate(detections):
             #   det = convert_to_cv2bbox(det)
                IOU_mat[t,d] = box_iou2(trk,det) 
        
        # Produces matches       
        # Solve the maximizing the sum of IOU assignment problem using the
        # Hungarian algorithm (also known as Munkres algorithm)
        
        matched_idx = linear_assignment(-IOU_mat)        

        unmatched_trackers, unmatched_detections = [], []
        for t,trk in enumerate(trackers):
            if(t not in matched_idx[:,0]):
                unmatched_trackers.append(t)

        for d, det in enumerate(detections):
            if(d not in matched_idx[:,1]):
                unmatched_detections.append(d)

        matches = []
       
        # For creating trackers we consider any detection with an 
        # overlap less than iou_thrd to signifiy the existence of 
        # an untracked object
        
        for m in matched_idx:
            if(IOU_mat[m[0],m[1]]<iou_thrd):
                unmatched_trackers.append(m[0])
                unmatched_detections.append(m[1])
            else:
                matches.append(m.reshape(1,2))
        
        if(len(matches)==0):
            matches = np.empty((0,2),dtype=int)
        else:
            matches = np.concatenate(matches,axis=0)
        
        return matches, np.array(unmatched_detections), np.array(unmatched_trackers)       
        


    def pipeline(self, z_box):
        '''
        Pipeline function for detection and tracking
        '''
        
        self.frame_count+=1
        
        x_box =[]
        
        if len(self.tracker_list) > 0:
            for trk in self.tracker_list:
                x_box.append(trk.box)
        
        
        matched, unmatched_dets, unmatched_trks \
        = self.assign_detections_to_trackers(x_box, z_box, iou_thrd = 0.3)  
        #print(matched, unmatched_dets, unmatched_trks) 
        # Deal with matched detections     
        if matched.size >0:
            for trk_idx, det_idx in matched:
                z = z_box[det_idx]
                z = np.expand_dims(z, axis=0).T
                tmp_trk = self.tracker_list[trk_idx]
                tmp_trk.kalman_filter(z)
                xx = tmp_trk.x_state.T[0].tolist()
                xx =[xx[0], xx[2], xx[4], xx[6]]
                x_box[trk_idx] = xx
                tmp_trk.box =xx
                tmp_trk.hits += 1
                tmp_trk.no_losses = 0
        
        # Deal with unmatched detections      
        if len(unmatched_dets)>0:
            for idx in unmatched_dets:
                z = z_box[idx]
                z = np.expand_dims(z, axis=0).T
                tmp_trk = kalman_tracker.Tracker() # Create a new tracker
                x = np.array([[z[0], 0, z[1], 0, z[2], 0, z[3], 0]]).T
                tmp_trk.x_state = x
                tmp_trk.predict_only()
                xx = tmp_trk.x_state
                xx = xx.T[0].tolist()
                xx =[xx[0], xx[2], xx[4], xx[6]]
                tmp_trk.box = xx
                self.tracker_list.append(tmp_trk)
                x_box.append(xx)
        
        # Deal with unmatched tracks       
        if len(unmatched_trks)>0:
            for trk_idx in unmatched_trks:
                tmp_trk = self.tracker_list[trk_idx]
                tmp_trk.no_losses += 1
                tmp_trk.predict_only()
                xx = tmp_trk.x_state
                xx = xx.T[0].tolist()
                xx =[xx[0], xx[2], xx[4], xx[6]]
                tmp_trk.box =xx
                x_box[trk_idx] = xx
                       
           
        # The list of tracks to be annotated  
        good_tracker_list =[]
        for trk in self.tracker_list:
            if ((trk.hits >= self.min_hits) and (trk.no_losses <=self.max_age)):
                 good_tracker_list.append(trk)

        # Book keeping
        self.deleted_tracks = filter(lambda x: x.no_losses > self.max_age, self.tracker_list)  
        
        
        self.tracker_list = [x for x in self.tracker_list if x.no_losses<=self.max_age]
    
def box_iou2(a, b):
    '''
    Helper funciton to calculate the ratio between intersection and the union of
    two boxes a and b
    a[0], a[1], a[2], a[3] <-> left, up, right, bottom
    '''
    
    w_intsec = np.maximum (0, (np.minimum(a[2], b[2]) - np.maximum(a[0], b[0])))
    h_intsec = np.maximum (0, (np.minimum(a[3], b[3]) - np.maximum(a[1], b[1])))
    s_intsec = w_intsec * h_intsec
    s_a = (a[2] - a[0])*(a[3] - a[1])
    s_b = (b[2] - b[0])*(b[3] - b[1])
  
    return float(s_intsec)/(s_a + s_b -s_intsec)


if __name__ == "__main__":    
    ros = ROS_runner()
    rospy.init_node('tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting Down")
