#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

#sub_image_topic = "/bebop/image_raw"
sub_image_topic = "/image_slow"
pub_image_topic = "/bb/image_box"
pub_bb_topic = "/yolo/bb_arr"
CONFIDENCE_THRESHOLD = 0.6

PATH = "/home/grammers/catkin_ws/src/time_to_collision_calculatin/yolo/src/"

class ROS_runner():
    def __init__(self):
       
        self.bridge = CvBridge()

        self.image_file_sub = rospy.Subscriber(
            sub_image_topic, Image, self.callback)
        
        self.bb_pub = rospy.Publisher(
            pub_bb_topic, BoundingBoxArray, queue_size = 10)

        self.image_pub = rospy.Publisher(
            pub_image_topic, Image, queue_size = 10)
        
        self.net = cv2.dnn.readNet(PATH + "yolov3.weights", PATH + "yolov3.cfg")
        self.classes = []
        with open(PATH + "coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0,255, size=(len(self.classes), 3))

        self.hight = 0
        self.width = 0
        self.channels = 0

    def box_extract(self, outs):
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > CONFIDENCE_THRESHOLD:
                    center_x = detection[0]
                    center_y = detection[1]
                    w = detection[2]
                    h = detection[3]
                    
                    x = center_x - w / 2
                    y = center_y - h / 2
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        return boxes, confidences, class_ids
    
    def vizualize(self, img, indexes, boxes, class_ids):
        font = cv2.FONT_HERSHEY_PLAIN
        
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                if label == "person":
                    color = self.colors[i]
                    cv2.rectangle(img, (x,y), (x + w, y + h), color, 2)
                    cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
    def to_msg(self, boxes, class_ids, header):
        box_arr = BoundingBoxArray()
        box_arr.header = header

        for i in range(len(boxes)):
            if str(self.classes[class_ids[i]]) == "person":
                x, y, w, h = boxes[i]
                box = BoundingBox()
                box.label = i
                box.value = 0
        
                box.pose.position.x = x
                box.pose.position.y = y
                box.pose.position.z = 0

                box.pose.orientation.x = 0 
                box.pose.orientation.y = 0 
                box.pose.orientation.x = 0 
                box.pose.orientation.w = 0 

                box.dimensions.x = w
                box.dimensions.y = h
                box.dimensions.z = 0

                box_arr.boxes.append(box) 

        return box_arr


    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.hight, self.width, self.channels = cv_image.shape

        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        boxes, confidences, class_ids = self.box_extract(outs)
        
        #indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        #self.vizualize(cv_image, indexes, boxes, class_ids)
        self.bb_pub.publish(self.to_msg(boxes, class_ids, data.header))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('yolo_detecter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
