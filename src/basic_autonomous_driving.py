#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import math


distance = 99


def move(velocity_publisher):
    DIST = 0.6
    global distance

    velocity_message = Twist()

    while distance > DIST:

        linear_speed = 0.3
        velocity_message.linear.x = linear_speed
        velocity_publisher.publish(velocity_message)

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(velocity_publisher):
    MAX_DIST = 1
    global distance

    velocity_message = Twist()

    while distance < MAX_DIST:

        linear_speed = 0
        angular_speed = 1.5
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

    velocity_message.angular.x = 0
    velocity_publisher.publish(velocity_message)


def scan_callback(scan_data):
    global distance
    distance = get_distance(scan_data.range)
    print('minumum distance: ', distance)


def get_distance(range):
    # ranges = [x for x in ranges if not math.isinf(x) or math.isnan(x)]

    return range


if __name__ == '__main__':

    # init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    rospy.Subscriber("/sonar", Range, scan_callback)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # time.sleep(2)
    while True:
        move(velocity_publisher)
        rotate(velocity_publisher)
