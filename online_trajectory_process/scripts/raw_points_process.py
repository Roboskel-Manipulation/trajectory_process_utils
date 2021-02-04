#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from std_msgs.msg import Bool

import numpy as np

pub = None
x, y, z = [], [], []
x_mov, y_mov, z_mov = [], [], []
init_flag, start = True, False
count = 0
end = False
num_points_std = 0
std_threshold = 0
outlier_dis = None

def callback(msg):
	global pub, x, y, z, init_flag, x_mov, y_mov, start, end, count, halt_motion
	global num_points_std, std_threshold, outlier_dis	
	count += 1
	# Get RWrist keypoint
	for i in range(len(msg.keypoints)):
		if msg.keypoints[i].name == "RWrist":
			x_point = msg.keypoints[i].points.point.x
			y_point = msg.keypoints[i].points.point.y
			z_point = msg.keypoints[i].points.point.z
			time_point = msg.keypoints[i].points.header.stamp
			if len(x) > 1:
				if abs(x[-1]-x_point) < outlier_dis and abs(y[-1]-y_point) < outlier_dis and abs(z[-1]-z_point) < outlier_dis:
					x.append(x_point)
					y.append(y_point)
					z.append(z_point)
			else:
				x.append(x_point)
				y.append(y_point)
				z.append(z_point)

	if init_flag:
		# Check for outliers or zeros (invalid trajectory points)
		if len(x_mov) == 0:
			x_mov.append(x_point)
			y_mov.append(y_point)
			z_mov.append(z_point)
		else:
			if x_point == 0.0 and y_point == 0.0 and z_point == 0.0:
				valid_point = False
			elif abs(x_mov[-1] - x_point) < outlier_dis and abs(y_mov[-1] - y_point) < outlier_dis and abs(z_mov[-1] - z_point) < outlier_dis:
				x_mov.append(x_point)
				y_mov.append(y_point)
				z_mov.append(z_point)
				valid_point = True
			else:
				valid_point = False

		# If valid trajectory point, check if the motion has started
		if len(x_mov) > 1 and valid_point:
			if len(x_mov) == num_points_std:
				del x_mov[0]
				del y_mov[0]
				del z_mov[0]
			std_x = np.std(x_mov)
			std_y = np.std(y_mov)
			std_z = np.std(z_mov)
			if (not start) and (std_x > std_threshold or std_y > std_threshold or std_z > std_threshold):
				rospy.loginfo("Motion started at sample %d"%count)
				rospy.loginfo('STD: %f, %f, %f'%(std_x, std_y, std_z))
				start = True
			if start:
				point = PointStamped()
				point.header.stamp = time_point
				point.point.x = x_point
				point.point.y = y_point
				point.point.z = z_point
				pub.publish(point)

def main():
	global pub, num_points_std, std_threshold, outlier_dis
	rospy.init_node("raw_points")
	num_points_std = rospy.get_param("raw_points/num_points_std", 25)
	std_threshold = rospy.get_param("raw_points/std_threshold", 0.01)
	outlier_dis = rospy.get_param("raw_poitns/outlier_dis", 0.1)
	sub = rospy.Subscriber("transform_topic", Keypoint3d_list, callback)
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10, latch=True)

	rospy.spin()

main()
