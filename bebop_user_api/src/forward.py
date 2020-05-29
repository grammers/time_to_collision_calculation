#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from simple_pid import PID

import math

heading_topic = '/bbox_avoid/heading'
angle_to_waypoint_topic = '/bbox_avoid/angle_waypoint'
waypoint_topic = '/bbox_avoid/waypoint'

P = 50.0
I = 1.0  #4.1
D = 1.6
FREQUENCY = 0.125

class ROS_runner():
    def __init__(self):
        self.heading_sub = rospy.Subscriber(
            heading_topic, Float32, self.callback)

        self.pose_sub = rospy.Subscriber(
            'bebop/odom', Odometry, self.waypoint)

        self.waypoint_sub = rospy.Subscriber(
            waypoint_topic, Point, self.waypoint)

        self.heading_pub = rospy.Publisher(
            '/bebop/cmd_vel', Twist, queue_size = 1)

        self.waypoint_pub = rospy.Publisher(
            angle_to_waypoint_topic, Float32, queue_size = 1)

        self.pid = PID(P, I, D, sample_time = FREQUENCY, output_limits=(-100,100))

        # desired heading
        self.angle_to_waypoint = 0.0
        # current heading
        self.yaw = 0.0
        # heading error
        self.error = 0.0

        # next waypoint
        self.waypoint_x = 10
        self.waypoint_y = 0
        self.waypoint_z = 0

    # get new waypoint
    def waypoint(self, data):
        self.waypoint_x = data.x
        self.waypoint_y = data.y
        self.waypoint_z = data.z


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
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
        
    # get odom update
    def waypoint(self, data):
        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pz = data.pose.pose.position.z
        
        ax = data.pose.pose.orientation.x
        ay = data.pose.pose.orientation.y
        az = data.pose.pose.orientation.z
        aw = data.pose.pose.orientation.w

        # get current orientation
        roll, pitch, self.yaw = self.quaterion_to_euler(ax, ay, az, aw)
        
        # avoid division with 0
        if self.waypoint_x - px == 0:
            g_yaw = 0
        else:
            g_yaw = math.atan((self.waypoint_y - py) / (self.waypoint_x - px))
        # calculate angle from current orientation to next waypoint
        d_yaw = self.yaw - g_yaw
        self.error = self.angle_to_waypoint - self.yaw 
        self.waypoint_pub.publish(d_yaw)
        #self.controller()


    def callback(self, data):
        self.angle_to_waypoint = data.data + self.yaw
        self.error = self.angle_to_waypoint - self.yaw

        self.controller()
    
    def controller(self):
        msg = Twist()

        # constant throttle ahead
        msg.linear.x = 0.05
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.pid(self.error) / 100 #+ D * (self.angle_to_waypoint - self.old_angle_to_waypoint) / FRESHENS
        #print("pid")
        #print self.error, msg.angular.z

        self.heading_pub.publish(msg)
        

if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('bebop_forward')

    try:
        #ros.controller()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
