#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simple_pid import PID

import math

heading_topic = '/heading'
goel_topic = '/goal'


P = 50.0
I = 1.0  #4.1
D = 1.6
FREQUENS = 0.2

class ROS_runner():
    def __init__(self):
        self.heading_sub = rospy.Subscriber(
            heading_topic, Float32, self.callback)

        self.pose_sub = rospy.Subscriber(
            'bebop/odom', Odometry, self.gole)

        self.heading_pub = rospy.Publisher(
            '/bebop/cmd_vel', Twist, queue_size = 1)

        self.gole_pub = rospy.Publisher(
            goel_topic, Float32, queue_size = 1)

        self.pid = PID(P, I, D, sample_time = FREQUENS, output_limits=(-100,100))

        self.target = 0.0
        self.yaw = 0.0
        self.error = 0.0

        self.goal_x = 10
        self.goal_y = 0

    def quaterion_to_euler(self, x,y,z,w):
        t0 = 2.0 * (w * x + y *z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0,t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < 1.0 else t2
        pitch = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(t3, t4)

        return roll, pitch
        
    def gole(self, data):
        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pz = data.pose.pose.position.z
        
        ax = data.pose.pose.orientation.x
        ay = data.pose.pose.orientation.y
        az = data.pose.pose.orientation.z
        aw = data.pose.pose.orientation.w

        roll, pitch = self.quaterion_to_euler(ax, ay, az, aw)
        
        if self.goal_x - px == 0:
            g_yaw = 0
        else:
            g_yaw = math.atan((self.goal_y - py) / (self.goal_x - px))
        d_yaw = self.yaw - g_yaw
        self.error = self.target - self.yaw 
        self.gole_pub.publish(d_yaw)
        self.controller()


    def callback(self, data):
        self.target = data.data + self.yaw
        self.error = self.target - self.yaw

    
    def controller(self):
        msg = Twist()

        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.pid(self.error) / 100 #+ D * (self.target - self.old_target) / FREQUENS
        #print("pid")
        #print self.error, msg.angular.z

        self.heading_pub.publish(msg)
        

if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('bebop_turet')

    try:
        #ros.controller()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
