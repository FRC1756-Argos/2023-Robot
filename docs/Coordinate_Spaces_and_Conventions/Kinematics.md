# Kinematics Coordinate Spaces

Refer to image `RobotFrames.png` to see visually where coordinate spaces are located

All coordinate spaces follow the WPILib "North West Up" convention. There is a coordinate space in the middle of the robot, which is the master, one in the middle of the shoulder axle, on the rotation axis, and one on the end of the arm, flush with the gear on the point where the wrist rotates.

The end effector will also have a coordinate space in the middle of it, or wherever we want kinematics treat the end effector's position

`ArmKinematics.png` Shows the basis of math behind the kinematic model.
