#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Scott Hadzik
from turtle import left
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

# Laser Parameters
ANGLE_RANGE = 360           # 360 degree scan
SCANS_PER_REVOLUTION = 360  # 1 degree increments
DISTANCE_TO_MAINTAIN = 1.5  # distance to stay from wall

vel_publisher = None
regions = {
    'right':0,
    'front_right':0,
    'front':0,
    'front_left':0,
    'left':0
}

state_ = 0
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall'
}

def change_state(state):
    global state_, state_dict
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict[state]))
        state_ = state

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2      # Move foward
    msg.angular.z = -0.3     # Turn Right
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3     # Turn Left
    return msg

def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.5      # Move foward
    return msg


def select_drive_state():
    global regions
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    
    d = DISTANCE_TO_MAINTAIN

    #================================= Front is Open ================================
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0) # Find the wall
    
    #================================= Wall at front================================
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 2 -front'
        change_state(1) # Turn Left
    
    #================================= Wall at front right ================================
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 3 - front right'
        change_state(2) # Follow the wall
    
    #================================= Wall at front left ================================
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 4 - front left'
        change_state(0) # Find the wall
    
    #================================= Wall at front and front right================================
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 5 - front and front right'
        change_state(1) # Turn Left
    
    #================================= Wall at front and front left ================================
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
        state_description = 'case 6 - front and front left'
        change_state(1) # Turn Left
    
    #================================= Wall at all front ================================
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 7 - front and front left and front right'
        change_state(1) # Turn Left
    
    #================================= Wall at front left and front right ================================
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
        state_description = 'case 8 - front left and front right'
        change_state(0) # Find the wall
    else:
        state_description = 'unknown state'
        rospy.loginfo(regions)

def laser_callback(data):
    global regions
    regions = {
        'front': data.ranges[0],                             # 0 degress
        
        'front_left':  min(min(data.ranges[40:50]), 10),     # 45 degress +/- 5 degrees
        'left':        min(min(data.ranges[85:95]), 10),     # 90 degress +/- 5 degrees
        
        'right':       min(min(data.ranges[265:275]), 10),   # 270 degress +/- 5 degrees
        'front_right': min(min(data.ranges[310:320]), 10)    # 315 degress +/- 5 degrees 
    }
    for position, range in regions.items():
        print(position, '\t\t', range)

    print('==================', '\n')
    print(data.ranges.index(min(data.ranges)), min(data.ranges))

def main():
    global vel_publisher
    
    rospy.init_node('wall_follow')
    laser_subscriber = rospy.Subscriber('/scan', LaserScan, laser_callback)
    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate=rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        if state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        else:
            rospy.loginfo('Unknown State!')

        vel_publisher.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    print('Wall following Starting')
    main()