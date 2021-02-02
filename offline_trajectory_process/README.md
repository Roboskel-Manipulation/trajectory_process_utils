# The offline trajectory process package
A ROS package for human movement detection and extra filtering of static points at the onset and end of the movement.

## Description

Given an input of 3D human joint positions (e.g. as recorded by [OpenPose](https://github.com/Roboskel-Manipulation/openpose_3D_localization), this package offers the following functionalities:
* Detection of the onset and end of the human movement
* Check for invalid positions or movements
* Filtering of redundant static points from the beginning and end of the movement 

NOTE: We use this package to provide to a UR3 cobot human instructed cartesian trajectories. 
## Functionality
* movement_detection.py: A node that detects, filters a dynamic movement.
	* movement detection: The detection of the movement onset is based on the standard deviation of the x, y and z coordinates. Using a sliding window of length N (movement_detection.yaml), if the standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion has started. The same criterion is used for the detection of the end of the motion. In the meantime, outliers are removed. Outliers are considered NaN values and values which correspond to erroneous measurements (points whose distance from the last valid point is greater than a predetermined threshold). If more than XX consecutive points satisfy the above condition the movement is considered invalid.
	*(optional) extra movement filtering: static_points_filtering_server (see below) is called to further clean the end of the movement.
	*(optional) movement smoothing: trajectory_smoothing_server is called to smooth the trajectory.
* static_points_filtering_server.py: A server for removing static redundant points from the beginning and the end of a recorded trajectory using the deviation from the median of an increased length window starting with N points (static_point_filtering.yaml).  It accepts three vectors containing the x, y, z coordinates in the form of a service request and returns the corresponding cleaned trajectory in the form of a service response. The declaration of the services are here.

## Launch files
To check the functionality real-time:
* Launch ...

* The arguments of the launch file are the following:
	*smooth: True if you want to smooth the trajectory (default false)
	*clean: True if you want to remove redundant points (default true)
