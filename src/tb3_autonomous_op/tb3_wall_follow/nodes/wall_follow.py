#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from threading import currentThread
# Author: Scott Hadzik
from turtle import left

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# Laser Parameters
ANGLE_RANGE = 360           # 360 degree scan
SCANS_PER_REVOLUTION = 222  # 1.6 degree increments
DISTANCE_TO_MAINTAIN = .3  # distance to stay from wall

vel_publisher = None
current_position = None

regions = {
    'right':0,
    'front_right':0,
    'front':0,
    'front_left':0,
    'left':0
}

wall_follow_state_ = 0
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall'
}

def change_state(wall_follow_state):
    global wall_follow_state_, wall_follow_state_dict
    if wall_follow_state is not wall_follow_state_:
        # print ('Wall follower - [%s] - %s' % (state, wall_follow_state_dict[state]))
        wall_follow_state_ = wall_follow_state

def find_wall(): #CASE 0
    msg = Twist()
    msg.linear.x = 0.15      # Move foward
    msg.angular.z = -0.35     # Turn Right
    return msg

def turn_left(): #CASE 1
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.35     # Turn Left
    return msg

def follow_the_wall():#CASE 2
    msg = Twist()
    msg.linear.x = 0.15      # Move foward
    msg.angular.z = 0.0     # no turning
    return msg   


def select_drive_state():
    global regions
    msg = Twist()
    linear_x = 0
    angular_z = 0

    wall_follow_state_description = ''
    
    d = DISTANCE_TO_MAINTAIN

    #================================= Front is Open ================================
    if regions['front'] > d and regions['front_left'] > d and regions['front_right'] > d:
        wall_follow_state_description = 'case 1 - nothing'
        change_state(0) # Find the wall by turning right
    
    #================================= Wall at front================================
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] > d:
        wall_follow_state_description = 'case 2 -front'
        change_state(1) # Turn Left
    
    #================================= Wall at front right ================================
    elif regions['front'] > d and regions['front_left'] > d and regions['front_right'] < d:
        wall_follow_state_description = 'case 3 - front right'
        change_state(1) # Turn Left
    
    #================================= Wall at front left ================================
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] > d:
        wall_follow_state_description = 'case 4 - front left'
        change_state(0) # Find the wall
    
    #================================= Wall at front and front right================================
    elif regions['front'] < d and regions['front_left'] > d and regions['front_right'] < d:
        wall_follow_state_description = 'case 5 - front and front right'
        change_state(1) # Turn Left
    
    #================================= Wall at front and front left ================================
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] > d:
        wall_follow_state_description = 'case 6 - front and front left'
        change_state(0) # Turn right
    
    #================================= Wall at all front ================================
    elif regions['front'] < d and regions['front_left'] < d and regions['front_right'] < d:
        wall_follow_state_description = 'case 7 - front and front left and front right'
        change_state(1) # Turn Left
    
    #================================= Wall at front left and front right ================================
    elif regions['front'] > d and regions['front_left'] < d and regions['front_right'] < d:
        wall_follow_state_description = 'case 8 - front left and front right'
        change_state(0) # Find the wall
    else:
        wall_follow_state_description = 'unknown state'
        rospy.loginfo(regions)
    print('wall_follow_state_description', wall_follow_state_description)

def laser_callback(data):
    global regions
    regions = {
        'front': data.ranges[0],                             # 0 degress
        
        'front_left':  min(min(data.ranges[32:38]), 10),     # 45 degress +/- 5 degrees
        'left':        min(min(data.ranges[60:66]), 10),     # 90 degress +/- 5 degrees 
        
        'right':       min(min(data.ranges[171:177]), 10),   # 270 degress +/- 5 degrees
        'front_right': min(min(data.ranges[198:204]), 10)    # 315 degress +/- 5 degrees 
    }
    # for position, range in regions.items():
        # print(position, '\t\t', range)
    # print('min',(data.ranges[np.nonzero(data.ranges)]))

    # print('ranges', len(data.ranges))

    select_drive_state()

def callback(msg):
    global current_position
    current_position = msg.pose.pose.position

def main():
    global vel_publisher
    state = 0
    
    rospy.init_node('wall_follow')
    laser_subscriber = rospy.Subscriber('/scan', LaserScan, laser_callback)
    pos_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():
                
        print(current_position)
        if current_position is not None:
            if current_position.x > 1.65:   #Wall follow
                print('wall follow')
                if state == 0:
                    print('wall follow')
                    msg = Twist()
                if wall_follow_state_ == 0:
                    msg = find_wall()
                elif wall_follow_state_ == 1:
                    msg = turn_left()
                elif wall_follow_state_ == 2:
                    msg = follow_the_wall()
                else:
                    rospy.loginfo('Unknown State!')
                vel_publisher.publish(msg)
            elif current_position.x < 1.65 and current_position.x > .85: # Line Follow
                print('line_follow')
            elif current_position.x < .85: # Go to goal
                print('go to goal')


        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    print('Wall following Starting')
    main()
