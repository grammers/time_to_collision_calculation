#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

pub_trace = "/bb/trace"
sub_boxes = "/yolo/bb_arr"
MAX_ERROR = 50

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

class ROS_runner():
    def __init__(self):
        self.boxes_sub = rospy.Subscriber(
            sub_boxes, BoundingBoxArray, self.callback)

        # acctual result
        self.trace_pub = rospy.Publisher(
            pub_trace, BoundingBoxArray, queue_size = 10)

        
        self.trace = []

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
        mini = MAX_ERROR
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

    def callback(self, data):
        box = []
        for boxes in data.boxes:
            box.append([boxes.pose.position.x, boxes.pose.position.y,
                boxes.pose.position.x + boxes.dimensions.x,
                boxes.pose.position.y + boxes.dimensions.y])

        self.det_to_trace(box)

        trace_message = self.msg_builder(data.header)
        
        self.trace_pub.publish(trace_message)

if __name__== '__main__':
    ros = ROS_runner()
    rospy.init_node('tracker', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shitting down")
    cv2.destroyAllWindows()
