#!/usr/bin/env python3

import rospy
import math
import cv2
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
# import the plan message
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist


sphere_params = SphereParams()

def get_params(SphereParams2):
    # get the sphere params from the subscriber
	global sphere_params
	sphere_params = SphereParams2


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_planner', anonymous = True)
	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# add a subscriber to sphere_params
	sphere = rospy.Subscriber('/sphere_params', SphereParams, get_params)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	
	
	
	# move robot to a spot where it is above the ball
	plan = Plan()
	plan_point1 = Twist()
	plan_point1.linear.x = -0.068
	plan_point1.linear.y = -0.669
	plan_point1.linear.z = 0.41
	plan_point1.angular.x = 3.01
	plan_point1.angular.y = 0.06
	plan_point1.angular.z = 2.822
	plan.points.append(plan_point1)
	
	# gather the XYZ coords of the balls center, adjust them to be consistent with the robot and move it there
	plan_point2 = Twist()
	plan_point2.linear.x = sphere_params.xc + 0.005
	plan_point2.linear.y = sphere_params.yc - 0.465
	plan_point2.linear.z = sphere_params.zc - 0.4633
	plan_point2.angular.x = 2.319
	plan_point2.angular.y = 0.041
	plan_point2.angular.z = 2.786
	plan.points.append(plan_point2)
	
	# move the robot back above the ball
	plan_point11 = Twist()
	plan_point11.linear.x = -0.068
	plan_point11.linear.y = -0.669
	plan_point11.linear.z = 0.41
	plan_point11.angular.x = 3.01
	plan_point11.angular.y = 0.06
	plan_point11.angular.z = 2.822
	plan.points.append(plan_point11)
	
	# move the robot holding the ball to a new location
	plan_point3 = Twist()
	plan_point3.linear.x = -0.449
	plan_point3.linear.y = -0.501
	plan_point3.linear.z = 0.412
	plan_point3.angular.x = 3.015
	plan_point3.angular.y = 0.06
	plan_point3.angular.z = 2.19
	plan.points.append(plan_point3)

	# put the ball down in new location 
	plan_point4 = Twist()
	plan_point4.linear.x = -0.455
	plan_point4.linear.y = -0.505
	plan_point4.linear.z = 0.057
	plan_point4.angular.x = 3.01
	plan_point4.angular.y = 0.06
	plan_point4.angular.z = 2.19
	plan.points.append(plan_point4)
	
	# pick arm back up and return to start
	plan_point31 = Twist()
	plan_point31.linear.x = -0.449
	plan_point31.linear.y = -0.501
	plan_point31.linear.z = 0.412
	plan_point31.angular.x = 3.015
	plan_point31.angular.y = 0.06
	plan_point31.angular.z = 2.19
	plan.points.append(plan_point31)
	

	
	
	
	while not rospy.is_shutdown():
		if sphere_params.xc > -0.02 and sphere_params.xc < 0.01:
			if sphere_params.yc > -0.025 and sphere_params.xc < 0.01:
				if sphere_params.zc > 0.46 and sphere_params.zc < 0.485:
					plan_pub.publish(plan)

		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
