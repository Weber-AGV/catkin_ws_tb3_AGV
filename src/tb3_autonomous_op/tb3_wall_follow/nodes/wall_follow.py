#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Scott Hadzik
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# class WallFollow():
#     def __init__(self) -> None:
#         self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
#         self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
#         self.sub_laser = rospy.Subscriber("/scan",LaserScan, 


#         # Laser Parameters
#         self.ANGLE_RANGE = 360
#         self.SCANS_PER_REVOLUTION = 1080


#         self.DISTANCE_RIGHT_THRESHOLD = 0.8 	# (m)
#         self.VELOCITY = 0.75 					# meters per second

def callback_laser(data):
    rospy.loginfo(data)


def main():
    rospy.init_node('wall_follow')

    sub = rospy.Subscriber('/ray', LaserScan, callback_laser)
    rospy.spin()




if __name__ == '__main__':
    # rospy.init_node('wall_follow')
    # node = WallFollow()
    main()