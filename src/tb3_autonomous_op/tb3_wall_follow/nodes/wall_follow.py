#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: Scott Hadzik
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Laser Parameters
ANGLE_RANGE = 360           # 360 degree scan
SCANS_PER_REVOLUTION = 360  # 1 degree increments


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

def laser_callback(data):
    global regions
    regions = {
        'right': min(min(data.ranges[80:100]), 10),         # 90  degress +/- 10 degrees
        'front_right': min(min(data.ranges[125:145]), 10),   # 125 degress +/- 10 degrees
        'front': min(min(data.ranges[170:190]), 10),        # 180 degress +/- 10 degrees
        'front_left': min(min(data.ranges[215:235]), 10),    # 225 degress +/- 10 degrees
        'left': min(min(data.ranges[260:280]), 10)          # 270 degress +/- 10 degrees
    }
    for region in regions:
        print(region)

    


# class WallFollow():
#     def __init__(self) -> None:
#         self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
#         self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
#         self.sub_laser = rospy.Subscriber("/scan",LaserScan, 


         # Laser Parameters
#         self.ANGLE_RANGE = 360
#         self.SCANS_PER_REVOLUTION = 360


#         self.DISTANCE_RIGHT_THRESHOLD = 0.8 	# (m)
#         self.VELOCITY = 0.75 					# meters per second


def main():
    rospy.init_node('wall_follow')
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.spin()




if __name__ == '__main__':
    print('Wall following Starting')
    # rospy.init_node('wall_follow')
    # node = WallFollow()
    main()