#!/usr/bin/env python3
# Import all necessary modules
import rospy
import math
import random
import numpy as np
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


XYZarray2 = XYZarray()


def get_xyz(XYZarray):
	global XYZarray2
	XYZarray2 = XYZarray
	

def process_data(XYZarray2):
	B = []
	A = []
	# Get the list of points from the XYZarray object
	point_list = XYZarray2.points
	
	# Fill A and B with all points, then solve for P
	for point in point_list:
		A.append([2*point.x, 2*point.y, 2*point.z, 1])
		B.append([point.x**2 + point.y**2 + point.z**2])
	P = np.linalg.lstsq(A, B, rcond = None)[0]
	
	return P


def filter_parameters(xc, yc, zc, radius):
	fil_in = xc
	fil_out = -0.014
	fil_gain = 0.05
	xc = fil_gain*fil_in + (1 - fil_gain)*fil_out
	
	fil_in = yc
	fil_out = -0.017
	fil_gain = 0.05
	yc = fil_gain*fil_in + (1 - fil_gain)*fil_out
	
	fil_in = zc
	fil_out = 0.475
	fil_gain = 0.05
	zc = fil_gain*fil_in + (1 - fil_gain)*fil_out
	
	fil_in = radius
	fil_out = 0.05
	fil_gain = 0.05
	radius = fil_gain*fil_in + (1 - fil_gain)*fil_out
	
	return SphereParams(xc, yc, zc, radius)
		


if __name__ == '__main__':
	# Define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# Define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# Define a publisher to publish the position and radius through SphereParams
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# Set the loop frequency
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():	
		# Exception handling to avoid empty/null data
		if len(XYZarray2.points) == 0:
			continue
		
		# Exception handling to ensure data being collected is accurate
		try:
			P = process_data(XYZarray2)
		except:
			print("Invalid Data")
			continue
		
		# Collect P and determine xc, yc and zc from it
		P = np.array(P)
		xc = P[0]
		yc = P[1]
		zc = P[2]
		# Uses np array P to calculate the radius of the ball
		radius = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
		
		# Filter the parameters
		sphere_params = filter_parameters(xc, yc, zc, radius)
		
		# Set up variable to publish, publish the data
		print(sphere_params.xc, sphere_params.yc, sphere_params.zc, sphere_params.radius)
		sphere_pub.publish(sphere_params)
		
		# Pause until the next iteration			
		rate.sleep()

