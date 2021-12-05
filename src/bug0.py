#THE LAUNCH FILE
#<!-- THE BUG0 ALGORITHM IMPLEMENTED FOR TURTLEBOT3 -->
#<launch>
#    <arg name="x_goal" default="2.0"/>
#    <arg name="y_goal" default="0.0"/>
#    <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_stage_4.launch"/>
#    <node name="follow_wall_server" pkg="bug0" type="follow_wall.py" output="screen"/>
#    <node name="go_to_goal_server" pkg="bug0" type="go_to_goal.py" output="screen"/>
#    <node name="bug0" pkg="bug0" type="main.py" output="screen">
#        <param name="x_goal" value="$(arg x_goal)" type="string"/>
#        <param name="y_goal" value="$(arg y_goal)" type="string"/>
#    </node>
#</launch>
#
#---------------------------------------------------------------------
#SERVICES
#UpdateGoal.srv
#geometry_msgs/Point32 goal
#---
#bool success
#-------------------------------
#SetVel.srv
#float32 lin_vel
#float32 ang_vel
#---
#bool success
#----------------------------------------------------------------------
#MESSAGES
#NavError.msg
#float32 yaw_error
#float32 dist_error
#----------------------------------------------------------------------
#SCRIPTS - MAIN DRIVER NODE
#!/usr/bin/env python

# main driver node
import rospy
from std_srvs.srv import SetBool, SetBoolRequest
from bug0.srv import UpdateGoal, UpdateGoalRequest
from bug0.msg import NavError
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from math import pi, cos, sin, fabs

# CONSTANTS
NODE_NAME = "main_driver"
# topics
SCAN_TOPIC = "scan"
ERR_TOPIC = "nav_error"
# services
FOLLOW_WALL_SRV = "fw/activate"
GO_TO_GOAL_SRV = "gtg/activate"
UPDATE_GOAL_SRV = "update_goal"
# navigation
DTOT = 0.4 # m
SIDE_CLEARANCE = 0.20
MAX_CLEARANCE = 0.7
LIN_THRSH = 0.04 # 4 cm

# GLOBALS
reached_goal: bool = False
following_wall: bool = False
going_to_goal: bool = False
# services
follow_wall: rospy.ServiceProxy = None
go_to_goal: rospy.ServiceProxy = None
update_goal: rospy.ServiceProxy = None
# navigation
goal = Point32()
yaw_err: float = None
front_clearance: float = None
# subscribers
scan_sub: rospy.Subscriber = None
err_sub: rospy.Subscriber = None


def clbk_err(msg: NavError) -> None:
    global yaw_err
    yaw_err = msg.yaw_error
    global reached_goal, front_clearance
    reached_goal = msg.dist_error < LIN_THRSH
    front_clearance = msg.dist_error if msg.dist_error < MAX_CLEARANCE else MAX_CLEARANCE


def map0to2pi(angle: float) -> float:
    """maps given angles in radians to corresponding angle in range from 0 to 2pi"""
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle


def look_for_obstacles(angle: float, scan: LaserScan, fov: float=pi) -> bool:
    f"""Returns False if any obstalces closer are found in the given direction"""
    fov /= 2
    end_ind = int(map0to2pi(angle + fov - scan.angle_min) / scan.angle_increment)
    start_ind = int(map0to2pi(angle - fov - scan.angle_min) / scan.angle_increment)
    angle = map0to2pi(-fov)
    if start_ind > end_ind:
        for r in scan.ranges[start_ind:]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < front_clearance and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment
        for r in scan.ranges[:end_ind]:
            x = r * cos(angle)
            y = fabs(r * sin(angle))
            if x < front_clearance and y < SIDE_CLEARANCE:
                return False
            angle += scan.angle_increment
    for r in scan.ranges[start_ind:end_ind]:
        x = r * cos(angle)
        y = fabs(r * sin(angle))
        if x < front_clearance and y < SIDE_CLEARANCE:
            return False
        angle += scan.angle_increment
    return True


def clbk_scan(scan: LaserScan) -> None:
    if not reached_goal:
        clear = look_for_obstacles(yaw_err, scan)
        global following_wall, going_to_goal
        if clear and following_wall:
            follow_wall.call(SetBoolRequest(data=False))
            go_to_goal.call(SetBoolRequest(data=True))
            going_to_goal = True
            following_wall = False
        elif not clear and going_to_goal:
            go_to_goal.call(SetBoolRequest(data=False))
            follow_wall.call(SetBoolRequest(data=True))
            going_to_goal = False
            following_wall = True
    

def setup() -> None:
    rospy.init_node(NODE_NAME)
    # setting up goal location
    goal.x = float(rospy.get_param("~x_goal"))
    goal.y = float(rospy.get_param("~y_goal"))
    # setting up service clients
    global follow_wall, go_to_goal, update_goal
    rospy.wait_for_service(FOLLOW_WALL_SRV)
    follow_wall = rospy.ServiceProxy(FOLLOW_WALL_SRV, SetBool)
    rospy.wait_for_service(GO_TO_GOAL_SRV)
    go_to_goal = rospy.ServiceProxy(GO_TO_GOAL_SRV, SetBool)
    rospy.wait_for_service(UPDATE_GOAL_SRV)
    update_goal = rospy.ServiceProxy(UPDATE_GOAL_SRV, UpdateGoal)
    # setting up subscribers
    global scan_sub, err_sub
    err_sub = rospy.Subscriber(ERR_TOPIC, NavError, clbk_err, queue_size=1)
    rospy.wait_for_message(ERR_TOPIC, NavError)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    update_goal.call(UpdateGoalRequest(goal=goal))
    rospy.sleep(5)
    go_to_goal.call(SetBoolRequest(data=True))
    global going_to_goal
    going_to_goal = True
    rospy.loginfo(f"\n---- {NODE_NAME} STARTING TO MOVE THE BOT TO :x= {goal.x} y={goal.y} ----\n")
    rospy.spin()


if __name__ == "__main__":
    main()

#! /usr/bin/env python
# 1. check if you have lost the wall
# 2. Find the wall and turn left
# 3. Follow the wall and if right region is free, turn right
# 4. loop

import rospy
from bug0.srv import SetVel, SetVelRequest, SetVelResponse
from math import pi, fabs, sin, cos, radians, isinf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from time import sleep

# CONSTANTS
NODE_NAME = "follow_wall_server"
# services
FW_SRV = "fw/activate"
SV_SRV = "fw/set_vel"
# topics
SCAN_TOPIC = "scan"
VEL_TOPIC = "cmd_vel"
# navigation
SIDE_CLEARANCE = 0.18
FRONT_CLEARANCE = 0.38
OFFSET = 0.032
DIST2WALL = 0.7


# GLOBALS
# states
machine_states = {"Inactive": True, "Finding Wall": False, "Turning Left": False, "Turning Right": False, "Following Wall": False}
# servers
fw_srvr: rospy.Service = None # follow wall server
sv_srvr: rospy.Service = None # set velocity server
# navigation
scan: LaserScan = None
vel = SetVelRequest()
vel.lin_vel = 0.13
vel.ang_vel = pi / 8
vel_cmd = Twist()
nt_obstacles = {}
regions = {}
lost_wall = True
# publishers
vel_pub: rospy.Publisher = None
# subscribers
scan_sub: rospy.Subscriber = None



def activate_handler(req: SetBoolRequest) -> SetBoolResponse:
    if machine_states["Inactive"] and not req.data:
        rospy.loginfo(f"\n---- {NODE_NAME} is already inactive! ----\n")
    elif machine_states["Inactive"]:
        rospy.loginfo(f"\n---- {NODE_NAME} has now been activated! ----\n")
        machine_states["Inactive"] = False
        machine_states["Finding Wall"] =  True
    elif not req.data:
        rospy.loginfo(f"\n---- {NODE_NAME} has now been inactivated! ----\n")
        machine_states["Inactive"] = True
        machine_states["Finding Wall"] = machine_states["Turning Left"] = machine_states["Turning Right"] = machine_states["Following Wall"] = False
    else:
        rospy.loginfo(f"\n---- {NODE_NAME} is already active! ----\n")
    return SetBoolResponse(success=True, message="Done!")


def sv_handler(req: SetVelRequest) -> SetVelResponse:
    vel.lin_vel = req.lin_vel
    vel.ang_vel = req.ang_vel
    SetVelResponse(success=True)


def map0to2pi(angle: float) -> float:
    pi_2 = pi * 2
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle += pi_2
    return angle


def clbk_scan(msg: LaserScan) -> None:
    global scan
    scan = msg
    
    lst_wall = is_clear = nt_obst = True
    x_comp = FRONT_CLEARANCE
    y_comp = SIDE_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    # angle = msg.angle_increment * index+ msg.angle_min
    angle = map0to2pi(-pi/4)
    start_ind = int(map0to2pi(angle - msg.angle_min) / msg.angle_increment)
    for r in msg.ranges[start_ind:]:
        x = r * cos(angle)
        y = -r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        elif x < x_comp1 and y < y_comp1:
            lst_wall = is_clear = False
        elif r < DIST2WALL:
            lst_wall = False
        angle += msg.angle_increment
    angle = 0
    end_ind = int(map0to2pi(pi / 4 - msg.angle_min) / msg.angle_increment)
    if lst_wall:
        for r in msg.ranges[:end_ind]:
            x = r * cos(angle)
            y = r * sin(angle)
            if x < x_comp and y < y_comp:
                lst_wall = is_clear = nt_obst = False
                break
            elif x < x_comp1 and y < y_comp1:
                lst_wall = is_clear = False
            elif r < DIST2WALL:
                lst_wall = False
            angle += msg.angle_increment
    elif is_clear:
        for r in msg.ranges[:end_ind]:
            x = r * cos(angle)
            y = r * sin(angle)
            if x < x_comp and y < y_comp:
                is_clear = nt_obst = False
                break
            elif x < x_comp1 and y < y_comp1:
                is_clear = False
            angle += msg.angle_increment
    elif nt_obst:
        for r in msg.ranges[:end_ind]:
            if r * cos(angle) < x_comp and r * sin(angle) < y_comp:
                nt_obst = False
                break
            angle += msg.angle_increment
    nt_obstacles["front"] = nt_obst
    regions["front"] = is_clear
    temp = True
    temp = lst_wall and temp
    start_ind = end_ind
    angle = pi / 4
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_min) / msg.angle_increment)
    lst_wall = is_clear = nt_obst = True
    x_comp = SIDE_CLEARANCE
    y_comp = FRONT_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = fabs(r * cos(angle))
        y = r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        elif x < x_comp1 and y < y_comp1:
            lst_wall = is_clear = False
        elif r < DIST2WALL:
            lst_wall = False
        angle += msg.angle_increment
    nt_obstacles["left"] = nt_obst
    regions["left"] = is_clear
    temp = lst_wall and temp
    start_ind = end_ind
    angle = pi / 4 + pi / 2
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_increment) / msg.angle_increment)
    lst_wall = is_clear = nt_obst = True
    x_comp = FRONT_CLEARANCE
    y_comp = SIDE_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = y_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = -r * cos(angle)
        y = fabs(r * sin(angle))
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        elif x < x_comp1 and y < y_comp1:
            lst_wall = is_clear = False
        elif r < DIST2WALL:
            lst_wall = False
        angle += msg.angle_increment
    nt_obstacles["back"] = nt_obst
    regions["back"] = is_clear
    temp = temp and lst_wall
    start_ind = end_ind
    angle = pi + pi / 4
    end_ind = int(map0to2pi(angle + pi / 2 - msg.angle_min) / msg.angle_increment)
    lst_wall = is_clear = nt_obst = True
    x_comp = SIDE_CLEARANCE
    y_comp = FRONT_CLEARANCE
    x_comp1 = x_comp + OFFSET
    y_comp1 = x_comp + OFFSET
    for r in msg.ranges[start_ind:end_ind]:
        x = fabs(r * cos(angle))
        y = -r * sin(angle)
        if x < x_comp and y < y_comp:
            lst_wall = is_clear = nt_obst = False
            break
        elif x < x_comp1 and y < y_comp1:
            lst_wall = is_clear = False
        elif r < DIST2WALL:
            lst_wall = False
        angle += msg.angle_increment
    nt_obstacles["right"] = nt_obst
    regions["right"] = is_clear
    global lost_wall
    lost_wall = temp and lst_wall


def stop_robot():
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    try:
        sleep(0.3)
    except rospy.ROSInterruptException:
        return


def find_wall():
    loop_rate = rospy.Rate(60)
    vel_cmd.linear.x = vel.lin_vel
    vel_cmd.angular.z = 0
    while nt_obstacles["front"] and nt_obstacles["right"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return
    machine_states["Finding Wall"] = False
    if not nt_obstacles["front"]:
        machine_states["Turning Left"] = True
    else:
        machine_states["Following Wall"] = True
    stop_robot()


def turn_left():
    loop_rate = rospy.Rate(60)
    vel_cmd.angular.z = vel.ang_vel
    vel_cmd.linear.x = 0
    while not nt_obstacles["front"] or regions["right"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return
    stop_robot()
    machine_states["Turning Left"] = False
    machine_states["Following Wall"] = True


def follow_wall():
    loop_rate = rospy.Rate(60)
    vel_cmd.linear.x = vel.lin_vel
    gain = 10
    ind1 = int(map0to2pi(radians(-80) - scan.angle_min) / scan.angle_increment)
    ind2 = int(map0to2pi(radians(-100) - scan.angle_min) / scan.angle_increment)
    while not regions["right"] and nt_obstacles["front"]:
        range1 = scan.ranges[ind1]
        range2 = scan.ranges[ind2]
        err = range2 - range1
        if range1 > DIST2WALL or range2 > DIST2WALL or fabs(err) > 0.6:
            vel_cmd.angular.z = 0
        else:
            vel_cmd.angular.z = gain * err
            if fabs(vel_cmd.angular.z) > vel.ang_vel:
                vel_cmd.angular.z = vel.ang_vel * (1 if err > 0 else -1)
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return
    machine_states["Following Wall"] = False
    if regions["right"]:
        machine_states["Turning Right"] = True
    else:
        machine_states["Turning Left"] = True
    stop_robot()


def turn_right():
    loop_rate = rospy.Rate(60)
    vel_cmd.angular.z = -vel.ang_vel
    vel_cmd.linear.x = 0
    while not nt_obstacles["front"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return
    OFFSET = 0.1
    while regions["front"]:
        vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            return
    OFFSET = 0.05
    stop_robot()
    machine_states["Finding Wall"] = True
    machine_states["Turning Right"] = False
    

def log_state():
    for key in machine_states:
        if machine_states[key]:
            rospy.loginfo(f"\n---- {NODE_NAME} is currently {key} ----\n")


def take_action():
    log_state()
    if machine_states["Finding Wall"]:
        find_wall()
    elif machine_states["Turning Left"]:
        turn_left()
    elif machine_states["Following Wall"]:
        follow_wall()
    elif machine_states["Turning Right"]:
        turn_right()


def setup():
    rospy.init_node(NODE_NAME)
    global fw_srvr, sv_srvr, vel_pub, scan_sub
    fw_srvr = rospy.Service(FW_SRV, SetBool, activate_handler)
    sv_srvr = rospy.Service(SV_SRV, SetVel, sv_handler)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, clbk_scan, queue_size=1)


def main():
    setup()
    loop_rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if not machine_states["Inactive"]:
            take_action()
        else:
            try:
                loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python

# go to goal

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from geometry_msgs.msg import Twist, Point32
import tf
from math import atan2, pi, radians, sqrt, fabs
from bug0.srv import UpdateGoalRequest, UpdateGoalResponse, UpdateGoal, SetVelResponse, SetVelRequest, SetVel
from bug0.msg import NavError
from sys import exit


# CONSTANTS
NODE_NAME = "gtg_server"
# services
GTG_SRV = "gtg/activate"
UG_SRV = "update_goal"
SET_VEL_SRV = "gtg/set_vel"
# topics
VEL_TOPIC = "cmd_vel"
ERR_TOPIC = "nav_error"
# frames
WORLD_FRAME = "odom"
ROBOT_FRAME = "base_footprint"
# navigation
GOAL = Point32()
ANG_THRESH = 3 * pi / 180 # 2 degrees
LIN_THRESH = 0.03 # 7 cm

# GLOBALS
# servers
gtg_srvr = None # go to goal server
ug_srvr = None  # update goal server
set_vel_srvr = None
# publishers
vel_pub = None
nav_err_pub = None
# subsribers / action clients
tf_listener = None
# messages
vel_cmd = Twist()
nav_err = NavError()
# navigation
cruising_vel = SetVelRequest()
cruising_vel.lin_vel = 0.20 # m/s
cruising_vel.ang_vel = 30 * pi / 180 # 30 deg/s
approaching_vel = SetVelRequest()
approaching_vel.lin_vel = cruising_vel.lin_vel * 0.5
approaching_vel.ang_vel = cruising_vel.ang_vel * 0.5
# booleans
active = False


def stop():
    vel_cmd.linear.x = vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)
    rospy.loginfo("\n --- STOPPING ROBOT ---\n")


def map0to2pi(angle: float) -> float:
    """"maps given angle to corresponding value from range 0 to 2 pi"""
    pi_2 = 2 * pi
    while angle > pi_2:
        angle -= pi_2
    while angle < 0:
        angle = pi_2 - angle
    return angle


def normalize_angle(angle: float) -> float:
    """maps given angle to corresponding value from range -pi to pi"""
    pi_2 = 2 * pi
    while angle > pi:
        angle -= pi_2
    while angle < -pi:
        angle += pi_2
    return angle


def update_error() -> None:
    translation, orientation = tf_listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time())
    if translation:
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        dx = GOAL.x - translation[0]
        dy = GOAL.y - translation[1]
        des_yaw = atan2(dy, dx)
        nav_err.yaw_error = normalize_angle(des_yaw - yaw)
        # TODO: replace normalize_angle() with map0to2pi()
        nav_err.dist_error = sqrt(dy * dy + dx * dx)
        nav_err_pub.publish(nav_err)


def correct_error():
    """Corrects yaw error"""
    if nav_err.dist_error < LIN_THRESH:
        rospy.loginfo("\n---- Reached Goal! x: {GOAL.x}, y: {GOAL.y} ----\n")
        stop()
        global active
        active = False
    dyaw = normalize_angle(nav_err.yaw_error)
    vel_cmd.angular.z = cruising_vel.ang_vel = cruising_vel.ang_vel * -1 if (cruising_vel.ang_vel > 0 and dyaw < 0) or (cruising_vel.ang_vel < 0 and dyaw > 0) else cruising_vel.ang_vel
    
    # start correcting for distance error only when angular error is Within limits
    if -ANG_THRESH < dyaw < ANG_THRESH:
        vel_cmd.angular.z = 0
        if nav_err.dist_error < 0.3:
            vel_cmd.linear.x = vel_cmd.linear.x * 0.95 + 0.05 * approaching_vel.lin_vel
        else:
            vel_cmd.linear.x = cruising_vel.lin_vel
    # slow down if error is about to diminish
    elif fabs(dyaw) < radians(15):
        vel_cmd.angular.z = 0.95 * vel_cmd.angular.z + 0.05 * approaching_vel.ang_vel
    else:
        vel_cmd.linear.x = 0


def reset() -> None:
    """resets the velocity controller
    * Doesn't change maximum velocities and gains"""
    stop()
    global active
    active = False


# handlers
def set_vel_handler(req: SetVelRequest) -> SetVelResponse:
    """sets the linear and angular velocities at which the robot will chase goal location"""
    global lin_vel, ang_vel
    lin_vel = req.lin_vel
    ang_vel = req.ang_vel
    return SetVelResponse(success=True)


def ug_handler(req: UpdateGoalRequest) -> UpdateGoalRequest:
    reset()
    GOAL.x = req.goal.x
    GOAL.y = req.goal.y
    GOAL.z = req.goal.z
    rospy.loginfo(f"\n---- GOAL UPDATED: x: {GOAL.x} y: {GOAL.y} ----\n")
    return UpdateGoalResponse(success=True)


def gtg_handler(req: SetBoolRequest) -> SetBoolResponse:
    global active
    if req.data and active:
        rospy.loginfo("gtg server already active!")
    elif req.data:
        reset()
        rospy.loginfo("\n---- gtg server now active! ----\n")
        active = req.data
    elif not req.data and not active:
        rospy.loginfo("gtg server already inactive!")
    else:
        rospy.loginfo("\n---- gtg server now inactive ----\n")
        active = req.data
        stop()
    return SetBoolResponse(success=True, message="Done!")


# setting up ros node
def setup():
    rospy.init_node(NODE_NAME)
    global gtg_srvr, ug_srvr, vel_pub, odom_sub, nav_err_pub, tf_listener, set_max_vel_srvr, set_gains_srvr
    gtg_srvr = rospy.Service(GTG_SRV, SetBool, gtg_handler)
    ug_srvr = rospy.Service(UG_SRV, UpdateGoal, ug_handler)
    set_max_vel_srvr = rospy.Service(SET_VEL_SRV, SetVel, set_vel_handler)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    tf_listener = tf.TransformListener()
    nav_err_pub = rospy.Publisher(ERR_TOPIC, NavError, queue_size=1)
    try:
       tf_listener.waitForTransform(ROBOT_FRAME, WORLD_FRAME, rospy.Time(), rospy.Duration(10))
    except rospy.exceptions.ROSInterruptException as e:
        exit(0)


def main():
    setup()

    loop_rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        update_error()
        if active:
            correct_error()
            vel_pub.publish(vel_cmd)
        try:
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()