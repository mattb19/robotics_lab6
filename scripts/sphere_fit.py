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

		


if __name__ == '__main__':
	# Define the node and subcribers and publishers
	rospy.init_node('sphere_params', anonymous = True)
	# Define a subscriber to get xyz of cropped ball
	img_sub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_xyz)
	# Define a publisher to publish the position and radius through SphereParams
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size=1)

	# Set the loop frequency
	rate = rospy.Rate(10)
	
	
	fil_outxc = -0.014
	fil_gainxc = 0.05
	
	
	fil_outyc = -0.017
	fil_gainyc = 0.05
	
	
	fil_outzc = 0.475
	fil_gainzc = 0.05
	
	
	fil_outrad = 0.05
	fil_gainrad = 0.05
	
	
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
		fil_inxc = P[0]
		fil_inyc = P[1]
		fil_inzc = P[2]
		# Uses np array P to calculate the radius of the ball
		fil_inrad = math.sqrt(P[3] + fil_inxc**2 + fil_inyc**2 + fil_inzc**2)
		
		# Perform calculations of the filter
		fil_outxc = fil_gainxc*fil_inxc + (1 - fil_gainxc)*fil_outxc
		fil_outyc = fil_gainyc*fil_inyc + (1 - fil_gainyc)*fil_outyc
		fil_outzc = fil_gainzc*fil_inzc + (1 - fil_gainzc)*fil_outzc
		fil_outrad = fil_gainrad*fil_inrad + (1 - fil_gainrad)*fil_outrad
		
		
		# Filter the parameters
		sphere_params = SphereParams(fil_outxc, fil_outyc, fil_outzc, fil_outrad)
		
		# Set up variable to publish, publish the data
		print(sphere_params.xc, sphere_params.yc, sphere_params.zc, sphere_params.radius)
		sphere_pub.publish(sphere_params)
		
		# Pause until the next iteration			
		rate.sleep()

