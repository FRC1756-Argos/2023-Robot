# Kinematics Coordinate Spaces

Refer to image `KinematicsCoordinates` to see visually where these are located

All coordinate spaces follow the WPILib "North West Up" convention. There is a coordinate space in the middle of the robot, which is the master, one in the middle of the shoulder axle, on the rotation axis, and one on the end of the arm, flush with the gear on the point where the wrist rotates.

The end effector will also have a coordinate space in the middle of it, or wherever we want inverse kinematics to originate from

# Control Conventions

For the shoulder:

Positive motor input should result in the shoulder raising the arm up, while negative motor input does the opposite.

For the arm extender:

Right stick up is arm extend out, while the inverse retracts it

For the wrist:

right stick right should turn the wrist clockwise, while the inverse should to counter-clockwise