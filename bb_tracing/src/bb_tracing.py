#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

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
sub_image_topic = "/bebop/image_raw"
pub_image_topic = "/image_box"
path = '/home/grammers/catkin_ws/src/nearCollision/data/'
conf_thres = 0.5
NMS_THRESH = 0.3

class Trace:
    ID = 0
    def __init__(self, box):
        Trace.ID += 1
        self.id = Trace.ID

        self.bbox = box

    def update(self, det):
        self.bbox = det

class ROS_runner:
    def __init__(self):

        self.bridge = CvBridge()
        
        self.trace = []

        #ros setup
        self.image_sub = rospy.Subscriber(
            sub_image_topic, Image, self.callback)

        self.image_pub = rospy.Publisher(
            pub_image_topic, Image, queue_size = 10)

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
                print(used_t)
                self.trace.pop(i)
                #delet trace

        for j, d in enumerate(dets):
            if used_d[j]:
                self.trace.append(Trace(d))
                #create trace


    def min_matrix(self, cost, used_t, used_d):
        trace_min = -1
        det_min = -1
        mini = 50
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

    def vis_detections(self, im, class_name, dets, time, thresh=0.5):
        inds = np.where(dets[:, -1] >= thresh)[0]
        if len(inds) == 0:
            return im
        
        box = []
        for i in inds:
            bbox = dets[i, :4]
            box.append(bbox)
            score = dets[i,-1]
            
            # bbox[x,y, x,y]
            cv2.rectangle(im, (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                (255, 255, 0), 2)
            
        self.det_to_trace(box)

        for t in self.trace:
            # bbox[x,y, x,y]
            cv2.rectangle(im, (int(t.bbox[0]), int(t.bbox[1])),
                (int(t.bbox[2]), int(t.bbox[3])),
                (255, 255, 0), 2)

            cv2.putText(im, str(t.id), (int(t.bbox[0]), int(t.bbox[3] - 2)),
                cv2.FONT_HERSHEY_SIMPLEX, int(1), (255, 0, 0), 
                int(2))
            


        return im



    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        scores, boxes = im_detect(self.sess, self.net, cv_image)

        cls_ind = 15 
        cls = 'person'
                
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
       
        dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        cv_image = self.vis_detections(cv_image, cls, dets, 0, thresh=conf_thres)


        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))



if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('bb_tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
