#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, Wrench
import math
import time
from nav_msgs.msg import Odometry


# def poseCallback(pose_message):
#     global x
#     global y, yaw
#     x = pose_message.pose.pose.position.x
#     y = pose_message.pose.pose.position.y
#     yaw = pose_message.pose.pose.position.z


def move(velocity_publisher, speed, distance, is_forward):

    velocity_message = Wrench()

    global x, y
    x0 = 0
    y0 = 0

    if (is_forward):
        velocity_message.force.x = abs(speed)
        # velocity_message.torque.z = abs(speed)

    else:
        velocity_message.force.x = -abs(speed)
        # velocity_message.torque.z = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(50)

    while True:
        rospy.loginfo("Robot moves forwards")
        velocity_publisher.publish(velocity_message)

        # loop_rate.sleep()

        # distance_moved = abs(math.sqrt(((0-x0) ** 2) + ((0-y0) ** 2)))
        # print("Distance moved :", distance_moved)
        # if not (distance_moved < distance):
        #     break

    # velocity_message.wrench.force.x = 0
    # velocity_publisher.publish(velocity_message)


# def go_to_goal(velocity_publisher, x_goal, y_goal):
#     global x
#     global y, yaw

#     velocity_message = Twist()

#     while (True):
#         K_linear = 0.3
#         distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

#         linear_speed = distance * K_linear

#         K_angular = 4.0
#         desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
#         angular_speed = (desired_angle_goal-yaw)*K_angular

#         velocity_message.linear.x = linear_speed
#         velocity_message.angular.z = angular_speed

#         velocity_publisher.publish(velocity_message)
#         print('x=', x, ', y=', y, ', distance to goal: ', distance)

#         if (distance < 0.01):
#             break


if __name__ == '__main__':
    try:

        rospy.init_node('first_motion_pose', anonymous=True)

        velocity_publisher = rospy.Publisher(
            "/argo/thruster_manager/input", Wrench, queue_size=10)

        # pose_subscriber = rospy.Subscriber(
        #     "/odom", Odometry, poseCallback)

        time.sleep(2)

        move(velocity_publisher, 1, 9.0, True)
        # go_to_goal(velocity_publisher, 3, 3)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
