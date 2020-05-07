#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

heading_topic = '/heading/'
P = 10
#I = 1
D = 1
FREQUENS = 2

class ROS_runner():
    def __init__(self):
        self.heading_sub = rospy.Subscriber(
            heading_topic, Int32, self.callback)


        self.heading_pub = rospy.Publisher(
            '/bebop/cmd_vel', Twist, queue_size = 1)
        
        self.target = 0
        self.target_old = 0

    def callback(self, data):
        self.old_target = self.target
        self.target = data.data

    
    def controller(self):
        msg = Twist()

        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.y = (self.target * P) / 4280 #+ D * (self.target - self.old_target) / FREQUENS
        

if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('bebop_turet')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
