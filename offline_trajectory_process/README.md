# offline_trajectory_process
A ROS package for detecting the onset and end of a human movement. Optionally, filtering of static points at the beginning and end of the trajectory and smoothing of the entire movement is also available.

## Description

Given an input of 3D human joint positions (e.g. as recorded by [OpenPose](https://github.com/Roboskel-Manipulation/openpose_3D_localization)), this package offers the following functionalities:
* Detection of the onset and end of the human movement
* Check for invalid positions or movements
* Filtering of redundant static points from the beginning and end of the movement 

NOTE: We use this package to provide to a UR3 cobot human instructed cartesian trajectories. 
## Functionality
* movement_detection.py: A node that detects, filters a dynamic movement.
	* movement detection: The detection of the movement onset is based on the standard deviation of the x, y and z coordinates. Using a sliding window of length N ([movement_detection.yaml](https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/config/movement_detection.yaml)), if the standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion has started. The same criterion is used for the detection of the end of the motion. In the meantime, outliers are removed. Outliers are considered NaN values and values which correspond to erroneous measurements (points whose distance from the last valid point is greater than a predetermined threshold). If more than XX consecutive points satisfy the above condition the movement is considered invalid.
	* (optional) extra movement filtering: static_points_filtering_server (see below) is called to further clean the end of the movement.
	* (optional) movement smoothing: [trajectory_smoothing_server](https://github.com/thanasists/trajectory_smoothing) is called to smooth the trajectory.
* static_points_filtering_server.py: A server for removing static redundant points from the beginning and the end of a recorded trajectory using the deviation from the median of an increased length window starting with N points ([static_point_filtering.yaml](https://github.com/ThanasisTs/trajectory_process_utils/blob/master/offline_trajectory_process/config/static_point_filtering.yaml)). It accepts three vectors containing the x, y, z coordinates in the form of a service request and returns the corresponding cleaned trajectory in the form of a service response. The declaration of the services are [here](https://github.com/ThanasisTs/trajectory_process_utils/tree/master/offline_trajectory_process/srv).

## Launch files
To check the functionality real-time:
* Launch
	* `roslaunch offline_trajectory_process static_point_filtering.launch`: Launch the static_point_filtering server
	* `roslaunch offline_trajectory_process movement_detection.launch`: Launch the movement_detection node

* The arguments of the `movement_detection.launch` file are the following:
	* smooth: True if you want to smooth the trajectory (default false)
