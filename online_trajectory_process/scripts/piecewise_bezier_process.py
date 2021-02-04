#!/usr/bin/env python
import rospy
from trajectory_custom_msgs.msg import PointArray
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from trajectory_process_utils_srvs.srv import *

import numpy as np
from scipy.spatial import distance

xV_tmp, yV_tmp, zV_tmp = [], [], []
x, y, z = [], [], []
xFinal, yFinal, zFinal = [], [], []
xRaw, yRaw, zRaw = [], [], []
xMov, yMov, zMov = [], [], []
count = 0
start_flag = False
num_points_std = None
std_threshold = None
outlier_dis = None
pub = None

def callback(data):
	global count, pub, xV_tmp, yV_tmp, zV_tmp, x, y, z
	global xFinal, yFinal, zFinal, xRaw, yRaw, zRaw, xMov, yMov, zMov
	global num_points_std, std_threshold, start_flag, outlier_dis

	# Get the RWrist keypoint
	for i in range(len(data.keypoints)):
		if (data.keypoints[i].name == "RWrist"):
			print ('received point')
			x_tmp = data.keypoints[i].points.point.x
			y_tmp = data.keypoints[i].points.point.y
			z_tmp = data.keypoints[i].points.point.z
			timestamp = data.keypoints[i].points.header.stamp.to_sec()
			break

	count += 1

	# if the motion has not ended, remove outliers and zeros (invalid trajectory points)
	if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
		if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < outlier_dis and abs(yRaw[-1] - y_tmp) < outlier_dis and abs(zRaw[-1] - z_tmp) < outlier_dis):
			xRaw.append(x_tmp)
			yRaw.append(y_tmp)
			zRaw.append(z_tmp)

			# If valid trajectory points, check if the motion has started
			if len(xV_tmp) == num_points_std:
				del xV_tmp[0]
				del yV_tmp[0]
				del zV_tmp[0]
			xV_tmp.append(x_tmp)
			yV_tmp.append(y_tmp)
			zV_tmp.append(z_tmp)
			if len(xV_tmp) >= 2:
				std_x = np.std(xV_tmp)
				std_y = np.std(yV_tmp)
				std_z = np.std(zV_tmp)
				if (not start_flag) and (std_x > std_threshold or std_y > std_threshold or std_z > std_threshold):
					print("Start movement at sample %d" %count)
					start_flag = True
				if start_flag:
					xMov.append(x_tmp)
					yMov.append(y_tmp)
					zMov.append(z_tmp)
					x.append(x_tmp)
					y.append(y_tmp)
					z.append(z_tmp)
					
					# If the motion has started, smooth the movement using 4 trajectory points with
					# one overlapping point (the final point of one quadraple is the first of the next)
					if len(x) == 4:
						try:
							rospy.wait_for_service("trajectory_smoothing")
							smoothing = rospy.ServiceProxy("trajectory_smoothing", Smoothing)
							resp = smoothing(x, y, z)
							x = resp.x_smooth
							y = resp.y_smooth
							z = resp.z_smooth
							x_all = resp.x_smooth_all
							y_all = resp.y_smooth_all
							z_all = resp.z_smooth_all
							
							rospy.loginfo("Smoothed the trajectory")
							xFinal.extend(x)
							yFinal.extend(y)
							zFinal.extend(z)
							if len(x) > 1:
								pub_rate = (3*0.047)/(len(x)-1)
							for i in xrange(1, len(x)):
								point = PointStamped()
								point.point.x = x[i]
								point.point.y = y[i]
								point.point.z = z[i]
								point.header.stamp = rospy.Time.now()
								pub.publish(point)
								rospy.sleep(pub_rate)
							
							end_time = rospy.get_time()
							x = [x[-1]]
							y = [y[-1]]
							z = [z[-1]]
						except rospy.ServiceException, e:
							rospy.logerr("Service call failed: %s"%e)	
					
def movement_detection_node():
	rospy.init_node("trajectory_process")
	global pub, num_points_std, std_threshold, outlier_dis
	rospy.loginfo("Ready to record NEW movement")
	num_points_std = rospy.get_param('trajectory_process/num_points_std', 25)
	std_threshold = rospy.get_param('trajectory_process/std_threshold', 0.01)
	outlier_dis = rospy.get_param("raw_poitns/outlier_dis", 0.1)	
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10, latch=True)	
	sub = rospy.Subscriber("transform_topic", Keypoint3d_list, callback)
	rospy.spin()


if __name__ == '__main__':
	movement_detection_node()

