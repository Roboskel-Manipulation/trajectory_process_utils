# The ur3 trajectory process package
A ROS package for processing a 3D cartesian trajectory to be executable by a UR3 Cobot.

## Description:
Given an input of 3D cartesian positions, this package offers the following functionalities:
* It maps the input trajectory points to end effector's positions based on the translation offset between the first point of the trajectory and the initial end effector's position.
* It checks if the mapped trajectory points satisfy the robot's limits, namely they do not lead to overextensions or self-collisions. If they do, these points are discarded.
* Online recalibration. During real-time teleoperation scenarios,  if [asked by the operator](https://github.com/thanasists/keypoints_relative_pos), the robot function can be halted and a new translation offset is recomputed based on the halted robot position and a new input trajectory point.
 
## Launch files
`roslaunch ur3_trajectory_process ur3_trajectory_process.launch`

### Arguments
* halt_motion: True to launch the [keypoints_relative_pos](https://github.com/thanasists/keypoints_relative_pos) node. Provides the functionality to halt the robot's motion (default: true)

