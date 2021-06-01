# online_trajectory_process
A ROS package for human movement onset detection and optionally smoothing of the ongoing movement. 

## Description
Given an input of 3D human joint positions (e.g. as recorded by [reference to Openpose_utils)], this package offers the following functionalities:
* Detection of the onset the human movement
* Check for invalid positions
* (Optional) Smoothing of the ongoing human movement. 
 
NOTE: We use this package to provide to a UR3 cobot human instructed cartesian positions during teleoperation. 


Functionality
* raw_points_process.py: A node that detects the onset of a human motion. The detection of the movement is based on the standard deviation of the x, y and z coordinates. Using a sliding window of length N, if the standard deviation of at least one coordinate is greater than a predefined threshold, then we assume that the motion has started. In the meantime, outliers are removed. Outliers are considered NaN values and values which correspond to erroneous measurements. Erroneous points are supposed to be points whose distance from the last valid point is greater than a predetermined threshold. 
* piecewise_bezier_process.py: This node offers the same functionalities to the raw_points_process.py node. Additionally,  it calls the [smoothing server](https://github.com/thanasists/trajectory_smoothing) for every four points. The last and first point of each quadruplet is the same. This results in a piecewise Bezier smoothed trajectory.
* downsampling_interpolation_process.py:  This node offers the same functionalities to the raw_points_process.py node. Additionally, it modifies the ongoing trajectory as follows: 
  * if the distance between two consecutive points is smaller than a predefined threshold Dmin, then the second point is discarded. 
  * when the distance between two consecutive points is greater than a predetermined threshold Dmax artificial points are interpolated across the line segment connecting these two points. These new points must be apart by at least Dmin.

This procedure yields a trajectory whose points are more equidistant than the raw one.

## Launch files
`roslaunch online_trajectory_process online_trajectory_process.launch`
### Arguments
* case_raw: True to launch `raw_points_process.py` node (default: false)
* case_bezier: True to launch `piecewise_bezier_process.py` node (default: false)
* case_downsampling: True to launch `downsampling_interpolation_process.py` node (default: false)


In the following plot, the resulting trajectory from the `piecewise_bezier_process` is shown in orange
<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/online_trajectory_process/PW_BZ.png" width="1000" height="500">


In the following plot, the resulting trajectory from the `downsampling_interpolation` is shown in red
<img src="https://github.com/ThanasisTs/trajectory_process_utils/blob/master/online_trajectory_process/DI.png" width="1000" height="500">
