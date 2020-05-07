#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

class ROS_runner():
    def __init__(self):
        self.land_pub = rospy.Publisher(
            'bebop/land', Empty, queue_size = 1)

        self.take_of_pub = rospy.Publisher(
            'bebop/takeoff', Empty, queue_size = 1)

        self.emergency = rospy.Publisher(
            'bebop/reset', Empty, queue_size = 1)

    def spin(self):
        while True:
            if rospy.is_shutdown():
                break

            command = raw_input()

            if command == "land":
                self.land_pub.publish()
            elif command == "takeoff":
                self.take_of_pub.publish()
            elif command == "reset" or command == "stop":
                self.emergency.publish()

if __name__ == "__main__":
    ros = ROS_runner()
    rospy.init_node('bebop_api')

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
