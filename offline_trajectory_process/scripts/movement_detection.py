#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from trajectory_custom_msgs.msg import PointStampedArray
from offline_trajectory_process.srv import *
from trajectory_smoothing.srv import *
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt

xRaw, yRaw, zRaw = [], [], []
xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
x, y, z, t = [], [], [], []
outliers_count = []
count = 0
movement_start = False
movement_end = False
movement_recording = True
invalid_movement = False
num_outliers, num_points_std, std_threshold = None, None, None
timenow = None
times = []
outlier_dis = None
marker = None
vis_human_pub = None

def callback(data, args):
	global timenow, xRaw, yRaw, zRaw, x, y, z, t, xV_tmp, yV_tmp, zV_tmp, tV_tmp, movement_start, movement_end, outlier_dis
	global count, movement_recording, outliers_count, invalid_movement, num_points_std, std_threshold, num_outliers
	global marker, vis_human_pub
	if movement_recording:

		# Comment this block of code if not using input of type geometry_msgs/Point 
		# x_tmp = data.x
		# y_tmp = data.y
		# z_tmp = data.z
		# timestamp = rospy.get_time()
		# count += 1
		# try:
		# 	# rospy.loginfo("Time duration: %f"%(rospy.Time.now().to_sec() - timenow))
		# 	times.append(rospy.Time.now().to_sec() - timenow)
		# except Exception as e:
		# 	rospy.logwarn(e)
		# timenow = rospy.Time.now().to_sec()

		# Uncomment this block of code when using input of type Keypoints_list
		


		for i in range(len(data.keypoints)):
			if (data.keypoints[i].name == "RWrist"):
				x_tmp = data.keypoints[i].points.point.x
				y_tmp = data.keypoints[i].points.point.y
				z_tmp = data.keypoints[i].points.point.z
				timestamp = data.keypoints[i].points.header.stamp.to_sec()
				count += 1
				break
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < outlier_dis and abs(yRaw[-1] - y_tmp) < outlier_dis and abs(zRaw[-1] - z_tmp) < outlier_dis):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
				if abs(x_tmp) < 0.6 and abs(y_tmp) < 0.6 and abs(z_tmp) < 0.6:
					if len(xV_tmp) == num_points_std:
						del xV_tmp[0]
						del yV_tmp[0]
						del zV_tmp[0]
						del tV_tmp[0]
					xV_tmp.append(x_tmp)
					yV_tmp.append(y_tmp)
					zV_tmp.append(z_tmp)
					tV_tmp.append(timestamp)
					if len(xV_tmp) >= 2:
						std_x = np.std(xV_tmp)
						std_y = np.std(yV_tmp)
						std_z = np.std(zV_tmp)
						# print(std_x, std_y, std_z)
						if (not movement_start) and (std_x > std_threshold or std_y > std_threshold or std_z > std_threshold):
							rospy.loginfo("Start movement at sample %d" %count)
							for k in xrange(len(xV_tmp)-num_points_std, len(xV_tmp)-1):
								x.append(xV_tmp[k])
								y.append(yV_tmp[k])
								z.append(zV_tmp[k])
								t.append(tV_tmp[k])
								marker_point = Point()
								marker_point.x = x[-1]
								marker_point.y = y[-1]
								marker_point.z = z[-1]
								marker.points.append(marker_point)
								vis_human_pub.publish(marker)
							movement_start = True
						if movement_start:
							x.append(x_tmp)
							y.append(y_tmp)
							z.append(z_tmp)
							t.append(timestamp)
							marker_point = Point()
							marker_point.x = x[-1]
							marker_point.y = y[-1]
							marker_point.z = z[-1]
							marker.points.append(marker_point)
							vis_human_pub.publish(marker)

							if std_x <= std_threshold and std_y <= std_threshold and std_z <= std_threshold:
								rospy.loginfo("End movement at sample %d" %count)
								movement_end = True
								movement_recording = False
			else:
				outliers_count.append(count)
				if len(outliers_count) == num_outliers and len(range(min(outliers_count), max(outliers_count)+1)) == num_outliers:
					rospy.logwarn("Invalid movement. Please record a new movement")
					invalid_movement = True
					movement_recording = False
	# else:
	# 	rospy.logerr("Cannot record movement")
					
def movement_detection_node():
	rospy.init_node("movement_detection_node")
	global times, x, y, z, t, xRaw, yRaw, zRaw, xV_tmp, yV_tmp, zV_tmp, tV_tmp, outlier_dis, movement_recording, movement_start, movement_end, count, invalid_movement, outliers_count, num_points_std, std_threshold, num_outliers
	global marker, vis_human_pub

	rospy.loginfo("Ready to record NEW movement")
	
	smooth_flag = rospy.get_param("movement_detection_node/smooth", False)
	filter_flag = rospy.get_param("movement_detection_node/filter", False)
	num_points_std = rospy.get_param("movement_detection_node/num_points_std", 24)
	std_threshold = rospy.get_param("movement_detection_node/std_threshold", 0.01)
	num_outliers = rospy.get_param("movement_detection_node/num_outliers", 10)
	outlier_dis = rospy.get_param("movement_detection_node/outlier_dis", 0.1)
	smooth_service_name = rospy.get_param("movement_detection_node/smooth_service_name", "trajectory_smoothing")
	filter_service_name = rospy.get_param("movement_detection_node/filter_service_name", "static_points_filtering")
	
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.action = marker.ADD
	marker.type = marker.LINE_STRIP
	marker.pose.position.x = 0
	marker.pose.position.y = 0
	marker.pose.position.z = 0
	marker.pose.orientation.x = 0
	marker.pose.orientation.y = 0
	marker.pose.orientation.z = 0
	marker.pose.orientation.w = 1
	marker.scale.x = 0.01
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.lifetime = rospy.Duration(100)
	
	marker_bezier = Marker()
	marker_bezier.header.frame_id = "base_link"
	marker_bezier.header.stamp = rospy.Time.now()
	marker_bezier.action = marker_bezier.ADD
	marker_bezier.type = marker_bezier.LINE_STRIP
	marker_bezier.pose.position.x = 0
	marker_bezier.pose.position.y = 0
	marker_bezier.pose.position.z = 0
	marker_bezier.pose.orientation.x = 0
	marker_bezier.pose.orientation.y = 0
	marker_bezier.pose.orientation.z = 0
	marker_bezier.pose.orientation.w = 1
	marker_bezier.scale.x = 0.01
	marker_bezier.color.a = 1.0
	marker_bezier.color.r = 0.0
	marker_bezier.color.g = 1.0
	marker_bezier.color.b = 0.0
	marker_bezier.lifetime = rospy.Duration(100)

	# Use the following subscription if you use the movement detection function
	# using Openpose and the custom message Keypoint3d_list
	sub = rospy.Subscriber('/transform_topic', Keypoint3d_list, callback, num_points_std)
	
	# Use the following subscription if you use the movement detection function
	# using a geometry_msgs/Point msg for each point
	# sub = rospy.Subscriber('raw_points', Point, callback, num_points_std)
	
	pub = rospy.Publisher('/candidate_points', PointStampedArray, queue_size=10)
	vis_human_pub = rospy.Publisher('/vis_human_topic', Marker, queue_size=10)
	vis_bezier_pub = rospy.Publisher('/vis_bezier_topic', Marker, queue_size=10)
	raw_pub = rospy.Publisher('/raw_movement_points', PointStampedArray, queue_size=10)
	x_raw, y_raw, z_raw, time_raw = [], [], [], []

	msg = PointStampedArray()
	while not rospy.is_shutdown():
		if (not movement_recording and not invalid_movement):
			times = []
			timenow = None
			for i in xrange(len(x)):
				x_raw.append(x[i])
				y_raw.append(y[i])
				z_raw.append(z[i])
				time_raw.append(t[i])
			if filter_flag:
				try:
					rospy.wait_for_service(filter_service_name)
					filtering = rospy.ServiceProxy(filter_service_name, Filtering)
					resp = filtering(x, y, z, t)
					x = resp.x
					y = resp.y
					z = resp.z
					t = resp.t

					rospy.loginfo("Filtered the points")
				except rospy.ServiceException, e:
					rospy.logerr("Cleaning service call failed: %s"%e)				

			# for i in xrange(len(x)):
			# 	point = PointStamped()
			# 	point.point.x = x[i]
			# 	point.point.y = y[i]
			# 	point.point.z = z[i]
			# 	point.header.stamp = rospy.Time.from_sec(t[i])
			# 	msg.points.append(point)
			# raw_pub.publish(msg)
			msg.points = []

			start_time = rospy.Time.now().to_sec()
			if smooth_flag:
				try:
					rospy.wait_for_service(smooth_service_name)
					smoothing = rospy.ServiceProxy(smooth_service_name, Smoothing)
					resp = smoothing(x, y, z)
					x = resp.x_smooth
					y = resp.y_smooth
					z = resp.z_smooth
					t = np.linspace(t[0], t[-1], len(x))
					for i in xrange(len(x)):
						point = Point()
						point.x = x[i]
						point.y = y[i]
						point.z = z[i]
						marker_bezier.points.append(point)
					vis_bezier_pub.publish(marker_bezier)
					rospy.loginfo("Smoothed the trajectory")
				except rospy.ServiceException, e:
					rospy.logerr("Smoothing service call failed: %s"%e)	
				
			rospy.loginfo('Bezier time duration: %f secs'%(rospy.Time.now().to_sec() - start_time))
			# fig = plt.figure()
			# ax = plt.axes()
			# ax.scatter(x_raw, y_raw, c='blue', s=20)			
			# ax.scatter(x, y, c='orange', s=20, label=len(x))
			# ax.set_xlabel('x(m)')
			# ax.set_ylabel('y(m)')
			# ax.grid()
			# ax.legend()
			# plt.show()
			x_raw, y_raw, z_raw = [], [], []

			for i in xrange(len(x)):
				point = PointStamped()
				point.point.x = x[i]
				point.point.y = y[i]
				point.point.z = z[i]
				point.header.stamp = rospy.Time.from_sec(t[i])
				msg.points.append(point)
			pub.publish(msg)
			msg.points = []
			movement_recording = True
			movement_start = False
			movement_end = False
			count = 0
			x, y, z, t = [], [], [], []
			xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
			xRaw, yRaw, zRaw = [], [], []
			outliers_count = []			
			rospy.sleep(1)
			rospy.loginfo("Ready to record NEW movement")

		if invalid_movement:
			rospy.loginfo("Sleep for 10 secs to setup the movement")
			rospy.sleep(10)
			invalid_movement = False
			movement_recording = True
			movement_start = False
			movement_end = False
			count = 0
			x, y, z, t = [], [], [], []
			xV_tmp, yV_tmp, zV_tmp, tV_tmp = [], [], [], []
			xRaw, yRaw, zRaw = [], [], []
			outliers_count = []
			rospy.loginfo("Ready to record NEW movement")

if __name__ == '__main__':
	movement_detection_node()
