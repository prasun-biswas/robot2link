# robot_two_link
# Robots used here are articulated arms with two joints. The first joint is at origin, around Y axis
(right-hand rule for rotating part and for coordinate system). The second joint is also around Y axis
but offset from origin by length0 on positive Z axis. Its zero position is so that the second linkage 
goes along positive X axis by length1. At that point is the end effector of the robot that we control.
There are two parameters that specify a robot, length0 and length1. Both joints have minimum and maximum 
positions and maximum velocities. Robots are specified in a file with the fields Length0 length1 min0 max0 maxvel0 min1 max1 maxvel1
