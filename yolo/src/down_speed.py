#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class ROS_runner:
    def __init__(self):
        self.counter = 0
        self.image_sub = rospy.Subscriber(
            '/bebop/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher(
            '/image_slow', Image, queue_size = 1)


    def callback(self, data):
        self.counter = (self.counter + 1) % 3
        if self.counter == 0:
            self.image_pub.publish(data)

def main():
    rospy.init_node('down_speeder', anonymous=True)
    ros_runner = ROS_runner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shoutting down')


if __name__ == '__main__':
    main()
