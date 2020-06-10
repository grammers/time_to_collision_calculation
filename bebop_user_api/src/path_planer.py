#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

waypoint_topic = '/bbox_avoid/waypoint'
presition = 0.5

class ROS_runner():
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            '/bebop/odom', Odometry, self.callback)

        self.waypoint_pub = rospy.Publisher(
            waypoint_topic, Point, queue_size = 1)

        self.path = []
        #self.path_builder(-5.0, 0.0, 1.3)
        #self.path_builder(0.0, 0.0, 1.3)
        #self.path_builder(0.0, 5.0, 1.3)
        #self.path_builder(0.0, 0.0, 1.3)
        self.path_builder(10.0, 0.0, 1.3)
        self.path_builder(0.0, 0.0, 1.3)
        #self.path_builder(0.0, -5.0, 1.3)
        #self.path_builder(0.0, 0.0, 1.3)
        self.index = 0

    def path_builder(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.path.append(point)

    def callback(self, data):
        curent_point = data.pose.pose.position
        print "pos"
        print curent_point
        print self.path[self.index]

        if curent_point.x < self.path[self.index].x + presition and curent_point.x > self.path[self.index].x - presition: 
            if curent_point.y < self.path[self.index].y + presition and curent_point.y > self.path[self.index].y - presition: 
                if curent_point.z < self.path[self.index].z + presition and curent_point.z > self.path[self.index].z - presition: 
                    self.index = (self.index + 1) % len(self.path)
        self.waypoint_pub.publish(self.path[self.index])


if __name__ == '__main__':
    ros = ROS_runner()
    rospy.init_node('path_planer')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting dows')
