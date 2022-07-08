#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Scott Hadzik
from turtle import left
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Laser Parameters
ANGLE_RANGE = 360           # 360 degree scan
SCANS_PER_REVOLUTION = 360  # 1 degree increments
DISTANCE_TO_MAINTAIN = 1.5  # distance to stay from wall


regions = {
    'right':0,
    'front_right':0,
    'front':0,
    'front_left':0,
    'left':0
}

state = 0
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall'
}

def select_drive_state():
    global regions
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    
    d = DISTANCE_TO_MAINTAIN

    
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 2 -front'
        change_state(1)
    
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
        state_description = 'case 3 - front right'
        change_state(2)
    
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)



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
    rospy.init_node('wall_follow')
    laser_subscriber = rospy.Subscriber('/scan', LaserScan, laser_callback)
    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate=rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state == 0:
            msg = find_wall()
        if state == 1:
            msg = turn_left()
        elif state == 2:
            msg = follow_the_wall()
        else:
            rospy.logger('Unknown State!')
        vel_publisher(msg)
        rate.sleep()


if __name__ == '__main__':
    print('Wall following Starting')
    main()