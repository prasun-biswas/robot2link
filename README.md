# robot_two_link
# Robots used here are articulated arms with two joints. The first joint is at origin, around Y axis
(right-hand rule for rotating part and for coordinate system). The second joint is also around Y axis
but offset from origin by length0 on positive Z axis. Its zero position is so that the second linkage 
goes along positive X axis by length1. At that point is the end effector of the robot that we control.
There are two parameters that specify a robot, length0 and length1. Both joints have minimum and maximum 
positions and maximum velocities. Robots are specified in a file with the fields Length0 length1 min0 max0 maxvel0 min1 max1 maxvel1

<img src="https://github.com/prasun-biswas/robot_two_link/blob/master/img/myrobot.gif" width="400" height="400" />

# 
The “read_robot_description()” method reads the description from “robot_description.txt” and checks if the description is valid, for example, the data type of the fields. Then, a robot is created and stored in the variable “robot_created” with the method “create_robot()”.  There are configurations for the robot’s initial pose. Then path description is read from “path_point.txt” with method “read_path_description()” and stored in the variable “work_path_points”. There are 3 major parts in generating the trajectory for the whole path. step 1: from initial position to work_start position, step 2: from work_start position to work_end position, step 3: from work_end position to initial position. The initial pose is set in configuration during initialization of the robot, “work_start” and “work_end” is generated from the “path_point.txt” with defined step_size. “step_size” is defined as ‘x’th division of unit ‘meter’. So, for creating path with 1mm step size along trajectory with method “find_point2point_trajectory()”, step_size will have a value of 1000 (1m/1000 = 1mm). 

#	
For each value in trajectory, the joint states are calculated with inverse kinematics. If all the points are within the workspace of the robot and reachable considering the joint rotation limitation then all the joint states for all positions are saved in the “solved_joint_states” variable. 

#
In the next stage, those joint state values are used to check if the velocity conditions are satisfied. Velocity condition is satisfied if the entire moves from point to point are below the maximum angular velocity limit.  If all of the above conditions are satisfied then final execution starts. Which includes the rotation of the motor and printing the joints as required in the task. For convenience, a graphic representation shows the movement of the robot besides printing the values. 

# 
The plotting is resource intensive when step size is 1 mm (1/1000) and slows down my computer. Please use a value of less than 100 throughout the program for the each call of function find_point2point_trajectory(). If you want to have a precision of 1 millimeter step size then execute the program with the plotting turned off by commenting out “robot.plot(pose)” inside the function execute_joint_poses(). However, in a powerful PC the program might work fine. The program was tested on a laptop with Intel Core i5 u-series processor with 6 GB RAM.
