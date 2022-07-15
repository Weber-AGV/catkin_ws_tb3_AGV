
Conversation opened. 14 messages. All messages read.

Skip to content
Using weber.edu Mail with screen readers
from:dkrawciw@uvic.ca 
Conversations

dkrawciw
dkrawciw@uvic.ca
Using 143.55 GB
Program Policies
Powered by Google
Last account activity: 0 minutes ago
Open in 1 other location · Details
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# based on https://github.com/experiencor/keras-yolo3


import roslib
import sys
import rospy
import math
import time
import cv2
import numpy as np
import cv_bridge
import os
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from std_msgs.msg import Float32
from std_msgs.msg import String
# ------------------
from object_recognition.msg import Predictor
import tensorflow as tf
from keras.preprocessing import image
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
from keras.models import load_model
# ------------------
import matplotlib.pyplot as plt
from matplotlib import patches, axes
import angles

# VELOCITY = 0 # (m/s)
VELOCITY = 0.04 # (m/s)

OFFSET = 1.5

errror = 0

sign_observed = False

distance_reached = False

stop_task_completed = False

first_odom_reading = False

xx = 0

yy = 0

x_stop = 0

y_stop = 0

x_init = 0

y_init = 0

GPU_OPTIONS = tf.compat.v1.GPUOptions(allow_growth=True)
CONFIG = tf.compat.v1.ConfigProto(gpu_options=GPU_OPTIONS)
CONFIG.gpu_options.per_process_gpu_memory_fraction = 0.5

sess = tf.compat.v1.Session(config=CONFIG)
tf.compat.v1.keras.backend.set_session(sess)

graph = tf.compat.v1.get_default_graph()
target_size = (416, 416)

model = load_model('/home/don/sae_ws/ros_ws/src/bootcamp-assignments/object_recognition/models/tinyyolo.h5', compile=False)

labels = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
  "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
  "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
  "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
  "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
  "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
  "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
  "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
        
bridge = CvBridge()

pub_msg = Predictor()

vel_msg = Twist()

bridge_object = CvBridge()

class Predictions:
  def __init__(self, objness = None, classes = None):
    self.objness = objness
    self.classes = classes
    self.label = -1
    self.score = -1
 
  def get_label(self):
    if self.label == -1:
      self.label = np.argmax(self.classes)

    return self.label
 
  def get_score(self):
    if self.score == -1:
      self.score = self.classes[self.get_label()]

    return self.score
 
def _sigmoid(x):
  return 1. / (1. + np.exp(-x))

def decode_netout(netout, anchors, obj_thresh, net_size):
  net_h = net_size[0]
  net_w = net_size[1]
  grid_h, grid_w = netout.shape[:2]
  nb_box = 3
  netout = netout.reshape((grid_h, grid_w, nb_box, -1))
  nb_class = netout.shape[-1] - 5
  
  pred_list = []
  netout[..., :2]  = _sigmoid(netout[..., :2])
  netout[..., 4:]  = _sigmoid(netout[..., 4:])
  netout[..., 5:]  = netout[..., 4][..., np.newaxis] * netout[..., 5:]
  netout[..., 5:] *= netout[..., 5:] > obj_thresh

  for i in range(grid_h*grid_w):
    row = i / grid_w
    col = i % grid_w
    for b in range(nb_box):
      # 4th element is objectness score
      objectness = netout[int(row)][int(col)][b][4]
      if(objectness.all() <= obj_thresh): continue
      # last elements are class probabilities
      classes = netout[int(row)][col][b][5:]
      pred = Predictions(objectness, classes)
      pred_list.append(pred)
  return pred_list
 
# get all of the results above a threshold
def get_predictions(preds, labels, thresh):
  
  v_labels, v_scores = list(), list()
  
  # enumerate all the predictions
  for pred in preds:
    # enumerate all possible labels
    for i in range(len(labels)):
      # check if the threshold for this label is high enough
      if pred.classes[i] > thresh:
        v_labels.append(labels[i])
        v_scores.append(pred.classes[i]*100)
  return v_labels, v_scores


def control_motors(cx, width):

#		Motor speed proportional controller

#		Inputs:
#			cx (float) - the x-coordinate of the masked image centroid
#			width (integer) - the width (along the x-axis) of the masked image

#		Outputs returned:
#			none

#   	To Do:
#			Do something with cy? - sometimes runs straight over the line  

	global vel_msg, velocity_publisher
	global errror
	Kp = 0.0005
		
#		car_msg = AckermannDriveStamped()
				

	error = (width/2) - cx
	errror = error


#		car_msg.drive.speed = VELOCITY
	vel_msg.linear.x = VELOCITY

	steering_angle = Kp*error 
#		steering_angle = 0
	if (steering_angle>0.5):
		steering_angle = 0.5
	elif (steering_angle <-0.5):
		steering_angle = -0.5

#		car_msg.drive.steering_angle= steering_angle
	vel_msg.angular.z = steering_angle
			
	velocity_publisher.publish(vel_msg)


#		self.car_pub.publish(car_msg)

#	for i in range(0,2000):
#		velocity_publisher.publish(vel_msg)
		
#	vel_msg.linear.x = 0
#	vel_msg.angular.z = 0
#	velocity_publisher.publish(vel_msg)


def stop_motors():

#	Stops Motors 

#	Outputs returned:
#		none

	global vel_msg, velocity_publisher
		
#		car_msg = AckermannDriveStamped()
				
	vel_msg.linear.x = 0
#	car_msg.drive.speed = VELOCITY

	vel_msg.angular.z = 0
#	car_msg.drive.steering_angle= steering_angle
		

	velocity_publisher.publish(vel_msg)
#	self.car_pub.publish(car_msg)
	
	time.sleep(.2)
		
	vel_msg.linear.x = 0
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)

def position_callback(data):
	global xx,yy, x_init, y_init, first_odom_reading,OFFSET
	xx = data.pose.pose.position.x
	yy = data.pose.pose.position.y
	
	if (first_odom_reading == False):
		x_init=xx
		y_init=yy
		first_odom_reading = True
		
#	print('Odom: %.1f, %.1f'% (xx,yy))
#	SPEED = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2 + data.twist.twist.linear.z**2)
#	return SPEED

#	plt.cla()
#	for stopping simulation with the esc key.
#	plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
#	plot_arrow(xc, yc, yaw)
#	plt.plot(x, cy, "-m", label="course")
	
	plt.plot(xx, yy, "sr")
	plt.axis([x_init-OFFSET,x_init+OFFSET,y_init-OFFSET,y_init+OFFSET])

#	plt.plot(target_x, target_y, "xg", label="target #"+str(idx_near_lookahead))
#	plt.plot(target_x, target_y, "xg", label="target #"+str(ibex))
#	plt.arrow(xc, yc, 1 * math.cos(yaw), 1 * math.sin(yaw),fc='k', ec='k', head_width=0, head_length=0)
#	plt.arrow(xc, yc, 1 * math.cos(desired_yaw), 1 * math.sin(desired_yaw),fc='r', ec='r', head_width=0.2, head_length=0.5)
#	plt.axis("equal")

	plt.grid(True)
	plt.title('Line Follower')

#	plt.title('with '+ str(len(waypoints)) +' Waypoints')
#	plt.legend(loc='upper center')
#	plt.draw

	plt.show	
	plt.pause(0.001)


def messy_callback(image_msg):

	global errror
	global sign_observed, distance_reached,stop_task_completed
	global xx,yy, x_stop, y_stop
    #First convert the image to OpenCV image 
#    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
	cv_image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
	height, width, channels = cv_image.shape  # height:480   width:640
    #print(cv_image.shape)


# -------------------------------
# Line Following

#       The following code takes a sliver of the image, a small set of rows about 40 pixels high 
#       and attempts to locate the center of the yellow patch of line within it. 

#      To implement this, first we get image dimensions and crop the parts of the image we don't need.

#		height, width, channels = cv_image.shape    # height:480   width:640
#		crop_img = cv_image[int(height/2)+100:int(height/2)+140][1:width]
	crop_img = cv_image[int(7*height/8):height][1:width]


#       Convert from RGB to HSV. Images defined in HSV schemes can be manipulated to be more robust
#        to light changes than with images defined in RGB.
	hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

#       Define a range of the color yellow.
#	lower_yellow = np.array([20,100,100])
#	upper_yellow = np.array([50,255,255])

#       Define a range of the color red.
#	lower_red = np.array([160,100,100])
#	upper_red = np.array([180,255,255])

#       Define a range of the color orange.
#	lower_orange = np.array([10,100,100])
#	upper_orange = np.array([25,255,255])

#       Define a range of the color green.
	lower_green = np.array([35,60,60])
	upper_green = np.array([85,255,255])


#      Threshold the HSV image to get only green colors
	mask = cv2.inRange(hsv, lower_green, upper_green)

#		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#		mask = cv2.inRange(hsv, lower_red, upper_red)
#		mask = cv2.inRange(hsv, lower_orange, upper_orange)


#		Calculate whether there is anything found in the masked image
	hist = cv2.calcHist([mask],[0],None,[256],[0,256])

	hht, wwd = mask.shape  # hht = height   wwd = width
	
	percent_black = hist[0]/(hht*wwd)		

#       Calculate centroid of the blob (cx, cy) of binary image using ImageMoments
	m = cv2.moments(mask, False)
	try:
		cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
	except ZeroDivisionError:
#			cx, cy = height/2, width/2
		cx, cy = width/2, height/2


#       Draw the centroid in the resultant image
	cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,0),-1)
		
#       Motor speed proportional controller 
#	control_motors(cx,width)

	if (percent_black > .98):
		print('No line observed')
		print(' ')
		stop_motors()

	else:
		print('I think I see a line')
		print(' ')
		control_motors(cx,width)
		



# -------------------------------
#  Object Recognition

	cv_image_target = cv2.resize(cv_image, target_size) 
	np_image = img_to_array(cv_image_target) #convert to numpy
	np_image = np_image.astype('float32')
	np_image /= 255.0
	np_image = np.expand_dims(np_image, 0)     # add a dimension so that we have one sample
	
	global sess
	global graph                                  # This is a workaround for asynchronous execution
	global model

	yhat = model.predict(np_image)            # Classify the image

	anchors = [[ 81,82,  135,169,  344,319],[10,14,  23,27,  37,58,]]

    # define the probability threshold for detected objects
	class_threshold = 0.1
    
	preds = list()
	for i in range(len(yhat)):
      # decode the output of the network
		preds += decode_netout(yhat[i][0], anchors[i], class_threshold, target_size)
        
    
    # get the details of the detected objects
	v_labels, v_scores = get_predictions(preds, labels, class_threshold)

    # summarize what we found
#	for i in range(len(v_labels)):
#		print(' ')
#		print('Howdy',v_labels[i], v_scores[i])
    
    # Publish only the max values
	try:
		max_pred_score = max(v_scores)
		max_pred_index = v_scores.index(max_pred_score)
		max_pred_label = v_labels[max_pred_index]

#		if (errror < 0):
#			print('\n\n\n*  Nice %s!  \033[38;5;1mTurning Right  \033[0;0m' % max_pred_label,end='\r\033[F\033[F\033[F')  #prints red
#		elif (errror>0):
#			print('\n\n\n*  Nice %s!  \033[38;5;2mTurning Left  \033[0;0m' % max_pred_label,end='\r\033[F\033[F\033[F')  #prints green
#		else: #errror = 0
#			print('\n\n\n*  Nice %s!  \033[38;5;7mGoing straight  \033[0;0m' % max_pred_label,end='\r\033[F\033[F\033[F')  #prints white

		pub_msg.header.stamp = rospy.Time.now()
		pub_msg.label = max_pred_label
		pub_msg.score = float(max_pred_score)
		pub_msg.box_coords = []
		pub.publish(pub_msg)
		
		if (max_pred_label == "stop sign"):

			if (sign_observed == False):
				x_stop = xx
				y_stop = yy
				print(x_stop, y_stop)
				sign_observed = True

			elif(sign_observed == True):

				distance = math.sqrt( (xx-x_stop)**2 + (yy-y_stop)**2)			
				print('distance: ',distance)
				if (distance < .4):
					distance_reached = False
				elif (distance >.4):
					distance_reached = True
			
				if (distance_reached == True) and (stop_task_completed == False):
					print('\a')
					print('\n\n\n Intense braking noise !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!', end='\r\033[F\033[F\033[F')
					stop_motors()
					time.sleep(3)
					stop_task_completed = True

	except ValueError:
#		print('\n\n\nPrediction below threshold', end='\r\033[F\033[F\033[F')
		pass


#      Display the manipulated image.
	cv2.imshow("Original", cv_image)
	cv2.moveWindow("Original",400,40)
	cv2.imshow("Original", cv_image)
	cv2.imshow("Mask", mask)
	cv2.moveWindow("Mask",400,int(7*height/8)+40)
	cv2.imshow("Mask", mask)
#	cv2.imshow("MASK", mask)
	cv2.waitKey(1)
	


rospy.init_node('ln_follow_obj_recog', anonymous=True)

print(' ')		
print(' ')		
print(' ')		
print(' ')		
print('***********************************************************')		
print('* SPOOKY LINE FOLLOWER with TINY YOLO OBJECT RECOGNITION  *')  		
print('***********************************************************')		
print(' ')		
print(' ')		
print(' ')		
print(' ')	
time.sleep(2)	
print('\a')	
os.system('clear')

rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, messy_callback, queue_size = 1, buff_size = 16777216)

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#car_pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)

pub = rospy.Publisher('object_detector', Predictor, queue_size = 10)

rospy.Subscriber("/odom", Odometry, position_callback, queue_size = 1)

while not rospy.is_shutdown():
  rospy.spin()


line_plus_tinyyolo.py
Displaying line_plus_tinyyolo.py.