#!/usr/bin/env python

import sys
sys.path.insert(0, '/home/grammers/catkin_ws/src/time_to_collision_calculatin/lib')
from bounding_box import Bounding_box

import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
#from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from vision_msgs.msg import Detection2DArray


#sub_image_topic = "/bebop/image_raw"
sub_image_topic = "/bbox_avoid/image_slow"
pub_image_topic = "/bb/image_box"
pub_bb_topic = "/bbox_avoid/detect"
CONFIDENCE_THRESHOLD = 0.5
CONF_SPLIT = 0.4

#print(cv2.__version__)

PATH = "/home/grammers/catkin_ws/src/time_to_collision_calculatin/yolo/net/"

class ROS_runner():
    def __init__(self):
       
        self.bridge = CvBridge()

        self.image_file_sub = rospy.Subscriber(
            sub_image_topic, Image, self.callback)
        
        self.bb_pub = rospy.Publisher(
            pub_bb_topic, Detection2DArray, queue_size = 10)

        self.image_pub = rospy.Publisher(
            pub_image_topic, Image, queue_size = 10)
        
        # net work sett upp
        self.net = cv2.dnn.readNet(PATH + "yolov3.weights", PATH + "yolov3.cfg")
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        self.classes = []
        with open(PATH + "my.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    # extracts bounding boxes form net.forward's output
    # img is sent with so it is needed to pass allong
    def box_extract(self, outs, img):
        confidences = []
        bounding_box = []
        for out in outs:
            for detection in out:
                # detection[5] == persion
                confidence = detection[5]
                if confidence > CONFIDENCE_THRESHOLD:
                    bbox = Bounding_box(img)
                    bbox.set_bbox(detection[0], detection[1], detection[2], detection[3])
                    bounding_box.append(bbox)
                    confidences.append(float(confidence))

        return bounding_box, confidences
    
    # remove dubel detectons
    def duble_removal(self, boxes, confidences):
        r_boxes = []
        for box in boxes:
            wx, ny = box.get_NW_corner()
            wx, ny = box.point_real(wx, ny)
            h = box.h_real()
            w = box.w_real()
            r_boxes.append([wx, ny, w, h])

        return cv2.dnn.NMSBoxes(r_boxes, confidences, CONFIDENCE_THRESHOLD, CONF_SPLIT)

    #not in use used for debuging
    def vizualize(self, img, indexes, boxes):
        font = cv2.FONT_HERSHEY_PLAIN
        
        for i in range(len(boxes)):
            #if i in indexes:
            if True:
                #x, y, w, h = boxes[i]
                wx, ny = boxes[i].get_NW_corner()
                ex, sy = boxes[i].get_SE_corner()
                wx, ny = boxes[i].point_real(wx, ny)
                ex, sy = boxes[i].point_real(ex, sy)
                #color = self.colors[i]
                cv2.rectangle(img, (int(wx), int(ny)), (int(ex), int(sy)), (255, 0,0), 2)
                #cv2.putText(img, label, (x, y + 30), font, 3, color, 3)

    def to_msg(self, boxes, header, indexes, img):
        box_arr = Detection2DArray()
        box_arr.header = header
        
        any_detect = False
        for i, box in enumerate(boxes):
            if i in indexes:
                box_arr.detections.append(box.to_msg())
                any_detect = True
        if not any_detect:
            box_arr.detections.append(Bounding_box(img).to_msg())
            

        return box_arr


    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #self.hight, self.width, self.channels = cv_image.shape
        
        #print(self.hight, self.width)

        # 320 or 416 or 608
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (608, 608), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        boxes, confidences = self.box_extract(outs, data)
        
        indexes = self.duble_removal(boxes, confidences)

        self.bb_pub.publish(self.to_msg(boxes, data.header, indexes, data))
        #self.vizualize(cv_image, indexes, boxes)
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('yolo_detecter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
