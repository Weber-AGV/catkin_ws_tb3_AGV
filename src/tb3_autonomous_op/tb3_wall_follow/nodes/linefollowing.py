#!/usr/bin/env python3
import roslib
import sys
import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3

VELOCITY = 0.04 # (m/s)

class LineFollower(object):
    def __init__(self):
       
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_projected_compensated",Image,self.camera_callback)
        # Create a move robot object in the constructor
        self.move_tb3 = MoveTurtlebot3()

    
    
    def camera_callback(self,data):

        cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="passthrough")


        # get image dimensions and crop the parts of the image we don't need.
        height, width, channels = cv_image.shape
        crop_img = cv_image[int(7*height/8):height][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Define a range of the color yellow.
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        hist = cv2.calcHist([mask],[0],None,[256],[0,256])

        hht, wwd = mask.shape  # hht = height   wwd = width
	
        percent_black = hist[0]/(hht*wwd)

        # Calculate centroid of the blob (cx, cy) of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = width/2, height/2

        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,0),-1)

        vel_msg = Twist()

        if (percent_black > .98):
            print('No line observed')
            print(' ')
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        
            self.move_tb3.move_robot(vel_msg)
            time.sleep(.2)
            
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.move_tb3.move_robot(vel_msg)
        else:
            print('I think I see a line')
            print(' ')
            Kp = 0.0005
            error = (width/2) - cx
            vel_msg.linear.x = VELOCITY
            steering_angle = Kp*error 

            if (steering_angle>0.5):
                steering_angle = 0.5
            elif (steering_angle <-0.5):
                steering_angle = -0.5
            vel_msg.angular.z = steering_angle
            self.move_tb3.move_robot(vel_msg)
                    
        









        # twist_object = Twist()
        # Kp = 0.0016667 # mt June 23 recordg 0.001666 # Kp
        # err = cx - width / 2 # control depends on error

        # twist_object.linear.x = 0.1 # mt June 23 recordg 0.1

        # twist_object.angular.z = -Kp * err    # P controller

        # self.move_tb3.move_robot(twist_object)


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
