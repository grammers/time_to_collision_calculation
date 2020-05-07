#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
#from bb_tracing.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray


import os
import sys
sys.path.insert(0, '/home/grammers/catkin_ws/src/tf-faster-rcnn/tools')
import _init_paths
from model.config import cfg
from model.test import im_detect
from model.nms_wrapper import nms

import tensorflow as tf
from nets.resnet_v1 import resnetv1

import numpy as np

import cv2
from cv_bridge import CvBridge

# theses should be lanch param
#sub_image_topic = "/bebop/image_raw"
sub_image_topic = "/image_slow"
sub_file_image_topic = "/jpg/image"
pub_image_topic = "/bb/image_box"
pub_trace = "/bb/trace"
path = '/home/grammers/catkin_ws/src/nearCollision/data/'
conf_thres = 0.5
NMS_THRESH = 0.3

# class list
# CLASSES = ('__background__',
#		   'aeroplane', 'bicycle', 'bird', 'boat',
#		   'bottle', 'bus', 'car', 'cat', 'chair',
#		   'cow', 'diningtable', 'dog', 'horse',
#		   'motorbike', 'person', 'pottedplant',
#		   'sheep', 'sofa', 'train', 'tvmonitor')

#cls_ind = 7 #car
cls_ind = 15 #human
#cls_ind = [7, 15]

class Trace:
    ID = 0
    def __init__(self, box):
        Trace.ID += 1
        self.id = Trace.ID

        self.bbox = box
        self.active = True

    def update(self, det):
        self.active = True
        self.bbox = det

    def to_msg(self):
        msg = BoundingBox()
        msg.label = self.id
        msg.value = 0
        
        msg.pose.position.x = self.bbox[0]
        msg.pose.position.y = self.bbox[1]
        msg.pose.position.z = 0

        msg.pose.orientation.x = 0 
        msg.pose.orientation.y = 0 
        msg.pose.orientation.x = 0 
        msg.pose.orientation.w = 0 

        msg.dimensions.x = self.bbox[2] - self.bbox[0]
        msg.dimensions.y = self.bbox[3] - self.bbox[1]
        msg.dimensions.z = 0
        
        #print (msg)

        return msg

class ROS_runner:
    def __init__(self):

        self.bridge = CvBridge()
        
        self.trace = []

        self.hight = 0
        self.width = 0

        #ros setup
        self.image_file_sub = rospy.Subscriber(
            sub_file_image_topic, Image, self.callback)
        self.image_sub = rospy.Subscriber(
            sub_image_topic, Image, self.callback)

        # visualisation
        self.image_pub = rospy.Publisher(
            pub_image_topic, Image, queue_size = 10)

        # acctual result
        self.trace_pub = rospy.Publisher(
            pub_trace, BoundingBoxArray, queue_size = 10)

        ### netwrk for fast rcnn ###
        cfg.TEST.HAS_RPN = True

        # set config
        self.tfconfig = tf.ConfigProto(allow_soft_placement=True)
        self.tfconfig.gpu_options.allow_growth=True

        #inint sesion
        self.sess = tf.Session(config=self.tfconfig)
        
        self.tfmodel = path + 'trained_models/voc_2007_trainval/res101_faster_rcnn_iter_70000.ckpt'
        # load box network
        self.net = resnetv1(num_layers=101)
        self.net.create_architecture("TEST", 21,
                              tag='default', anchor_scales=[8, 16, 32])
        self.saver = tf.train.Saver()
        self.saver.restore(self.sess, self.tfmodel)
        ### end ###

    def msg_builder(self, header):
        msg = BoundingBoxArray()
        msg.header = header
        
        for t in self.trace:
            if t.active:
                msg.boxes.append(t.to_msg())
            
        return msg

    def det_to_trace(self, dets):

        if len(self.trace) == 0:
            for d in dets:
                self.trace.append(Trace(d))
            return

        cost = [[50 for j in range(len(self.trace))] for i in range(len(dets))] 
        i = 0
        j = 0
        for t in self.trace:
            j = 0
            for d in dets:
                cost[j][i] = abs(t.bbox[0] - d[0]) + abs(t.bbox[1] - d[1]) + abs(t.bbox[2] - d[2]) + abs(t.bbox[3] - d[3]) 

                j +=1
            i +=1

        # find min val in cost
        # assign that dets to trace
        # lim max of.

        # 2 x bol array dets and trace
        # set false wen asined

        # for true trace delet
        # for true dets create trace
        used_t = [True for t in self.trace]
        used_d = [True for d in dets]
    
        for t in self.trace:
            min_i, min_j = self.min_matrix(cost, used_t, used_d)
            if min_i != -1:
                (self.trace[min_i]).update(dets[min_j])
                used_t[min_i] = False
                used_d[min_j] = False
        
        i = len(self.trace)
        while i > 0:
            i -= 1
            if used_t[i]:
                if self.trace[i].active:
                    self.trace[i].active = False
                else:
                    self.trace.pop(i)
                #delet trace

        for j, d in enumerate(dets):
            if used_d[j]:
                self.trace.append(Trace(d))
                #create trace


    def min_matrix(self, cost, used_t, used_d):
        trace_min = -1
        det_min = -1
        mini = 30
        for i in range(len(used_t)):
            for j in range(len(used_d)):
                if i >= len(cost):
                    break
                if j >= len(cost[0]):
                    break

                if cost[i][j] < mini and used_t[i] and used_d[j]:
                    mini = cost[i][j]
                    trace_min = i
                    det_min = j

        return trace_min, det_min

    def normalize(self, dets):
        #print self.width, self.hight
        for d in dets:
            d[0] = d[0] / self.width
            d[1] = d[1] / self.hight
            d[2] = d[2] / self.width
            d[3] = d[3] / self.hight
        return dets

    def zero_clean(self):
        i = len(self.trace)
        while i > 0:
            i -= 1
            if self.trace[i].active:
                self.trace[i].active = False
            else:
                self.trace.pop(i)
    

    def detections(self, dets, thresh=0.5):
        inds = np.where(dets[:, -1] >= thresh)[0]
        if len(inds) == 0:
            self.zero_clean()
            return 
        
        box = []
        for i in inds:
            bbox = dets[i, :4]
            box.append(bbox)
            score = dets[i,-1]
            
            # bbox[x,y, x,y]
            #cv2.rectangle(im, (int(bbox[0]), int(bbox[1])),
            #    (int(bbox[2]), int(bbox[3])),
            #    (255, 255, 0), 2)
            
        box = self.normalize(box)
        self.det_to_trace(box)


    def visualize(self, im):
        for t in self.trace:
            print("error")
            # bbox[x,y, x,y]
            #cv2.rectangle(im, (int(t.bbox[0] * self.width), int(t.bbox[1] * self.hight)),
            #    (int(t.bbox[2] * self.width), int(t.bbox[3] * self.hight)),
            #    (255, 255, 0), 2)

            #cv2.putText(im, str(t.id), (int(t.bbox[0]), int(t.bbox[3] - 2)),
            #    cv2.FONT_HERSHEY_SIMPLEX, int(1), (255, 0, 0), 
            #    int(2))
            
        return im


    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hight, self.width, channels = cv_image.shape
        
        ## detecton network
        scores, boxes = im_detect(self.sess, self.net, cv_image)
        
        #cls = 'person'
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
       
        dets =  np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        self.detections(dets, thresh=conf_thres )
        ## 

        #self.visualize(cv_image)

        trace_message = self.msg_builder(data.header)
        
        self.trace_pub.publish(trace_message)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))



if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('bb_tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
