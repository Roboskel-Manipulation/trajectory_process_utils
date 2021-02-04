#!/usr/bin/env python
import rospy
from offline_trajectory_process.srv import *
from statistics import median


num_points_median = None
thresX = None
thresY = None
thresZ = None

def handle_filtering(req):
	global num_points_median, thresX, thresY, thresZ
	x = req.x
	y = req.y
	z = req.z
	t = req.t

	for i in xrange(num_points_median, len(x)-1):
		if abs(x[i] - median(x[0:i])) > thresX or abs(y[i] - median(y[0:i])) > thresY or abs(z[i] - median(z[0:i])) > thresZ:
			break

	for j in xrange(len(x)-num_points_median, 1, -1):
		if abs(x[j] - median(x[j:len(x)])) > thresX or abs(y[j] - median(y[j:len(y)])) > thresY:
			break

	x = x[i-3:j+3]
	y = y[i-3:j+3]
	z = z[i-3:j+3]
	t = t[i-3:j+3]
	return FilteringResponse(x, y, z, t)


def static_points_filtering_server():
	rospy.init_node("static_points_filtering_server")
	global num_points_median, thresX, thresY, thresZ
	num_points_median = rospy.get_param("/static_points_filtering_server/num_points_median", 12)
	thresX = rospy.get_param("/static_points_filtering_server/thresX", 0.012)
	thresY = rospy.get_param("/static_points_filtering_server/thresY", 0.012)
	thresZ = rospy.get_param("/static_points_filtering_server/thresZ", 0.012)
	s = rospy.Service('static_points_filtering', Filtering, handle_filtering)
	rospy.loginfo("Ready to remove redundant points")
	rospy.spin()



if __name__=="__main__":
	static_points_filtering_server()