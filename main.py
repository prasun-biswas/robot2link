"""
author@ Prasun Biswas
"""
import math
import sys, getopt, time
import numpy as np
from roboticstoolbox import *
from spatialmath import SE3
from collections import OrderedDict
import multiprocessing
import motor


class R2Link(DHRobot):
    """
        This solution is built with roboticstoolbox library.

        R2Link class is written by inheriting DHRobot class from roboticstoolbox library.
        Class for robots defined using Denavit-Hartenberg notation
        using DHRobot class robot is configured. This library allows to build robot with
        joint angle rotation limitation. Also, this library calculates forward and inverse
        kinematics for defined robot configuration.

    """

    def __init__(self, symbolic=False, length0=1.5, length1=1.0, min0=-90.0, max0=90.0, maxvel0=1.0, min1=-120,
                 max1=120, maxvel1=1.0):
        """

        :param length0: length of first link in meter
        :param length1: length of second link in meter
        :param min0: min rotation angle of j0 in degree
        :param max0: max rotation angle of j0 in degree
        :param maxvel0: maximum angular velocity
        :param min1: min rotation angle of j1 in degree
        :param max1: max rotation angle of j1 in degree
        :param maxvel1: maximum rotation velocity
        """

        self.length0 = length0
        self.length1 = length1
        self.min0 = min0
        self.max0 = max0
        self.maxvel0 = maxvel0
        self.min1 = min1
        self.max1 = max1
        self.maxvel1 = maxvel1

        print(f"\nvalues for R2Link robot initilization: \n {self.length0}, {self.length1},"
              f" {self.min0},{self.max0}, {self.maxvel0}, "
              f"{self.min1}, {self.max1}, {self.maxvel1}")

        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0
        deg = pi / 180
        base = 0

        L = [
            RevoluteDH(d=base,
                       a=self.length0,
                       alpha=0.0,
                       offset=0.0,
                       qlim=[self.min0, self.max0],
                       flip=False),
            RevoluteDH(d=base,
                       a=self.length1,
                       alpha=0.0,
                       offset=0.0,
                       qlim=[self.min1, self.max1],
                       flip=False),
            RevoluteDH(d=0,
                       a=0.0,
                       alpha=0.0,
                       offset=0.0,
                       qlim=None,
                       flip=False)
        ]

        super().__init__(L,
                         name="Robot 2 link",
                         keywords=('planar',)
                         )
        # self.addconfiguration("qz",[1.5708,-1.5708])
        # the configuration creates different poses for robot using joint angle.
        # for this solution default starting pose is qz2: [1.5708, -1.5708, 0.0]
        self.addconfiguration("qz", [0, 0, 0])
        self.addconfiguration("qz1", [1, 1, 0])
        self.addconfiguration("qz2", [1.5708, -1.5708, 0.0])
        self._servo_j0 = motor.servo_motor(0, 'j0')
        self._servo_j1 = motor.servo_motor(0, 'j1')

    def solve_reachable_ikin(self, Ts):
        """
            this method checks if all the Transformation matrix is reachable by the
            configured robot. inverse kinematics to find the valid poses are calculated
            by using ikine_LM method of DHRobot class inherited is used to solve for
            reachable targets. Ikine_LM uses Numerical inverse kinematics by
            Levenberg-Marquadt optimization(Robot superclass)

        :param Ts: trajectory for a point to point movements
        :return: solved joint states if all points are reachable
        """
        sol = self.ikine_LM(Ts)

        print("ikine Solution: \n")
        set_a = set(sol.success)
        print(set_a)
        if False in set_a:
            count_false = np.count_nonzero(sol.success == False)
            itemindex = np.where(sol.success == False)
            print(f"unreachable goal:{count_false} at {itemindex[0]}")
            if count_false == 1 and itemindex[0] == 0:
                # sometimes there are zero values at the first position of array which results alse
                # f warning of unreachable goal. As a bug fix the value is deleted from numpy array

                print("false warning at first position")
                q_modified = np.delete(sol.q, 0, 0)
                print("(q_modified) all pathpoints are reachable: \n")
                return q_modified

        else:
            print("all pathpoints are reachable")
            # sol.q has all the solved valid poses. other un-necessary values are ignored
            return sol.q
        # returns empty if inverse kinematics  fails
        return []

    def plot_movement(self, joint_states):
        """
        this plots the movements of the configured robot in 3d space
        :param joint_states: all the joint states the robot need to execute
        :return:
        """
        print("received joint states to show: \n")
        for item in joint_states:
            # print(item)
            self.plot(item)
            time.sleep(0.001)
        self.plot(joint_states[-1]).hold()

    def chk_joint_vel_satisfied(self, joint_states, speed: float = 1.0):
        """
            This method checks if the robot's angular velocity limitation allows the robot to execute all the
            movements along trajectory for all joints along all the points at a certain speed. The process is
            finds the angular difference between 2 positions and the speed for each steps.

        :param joint_states:
        :param speed:
        :return:
        """
        time_step = 1 / speed
        failed_vel_at_step = OrderedDict()

        print(f"\nchecking joint vel condition: maxvel_j0: {self.maxvel0}, maxvel_j1: {self.maxvel1} \n  ")

        def diff_angle(x, y, joint='joint'):
            PI = math.pi
            angle = min((2 * PI) - abs(x - y), abs(x - y))
            # print(f"angle diff of {joint}: rad {angle} or deg: {np.rad2deg(angle)}")
            return angle

        # print("printing joint states for j1 and j2:  \n")
        # for item in joint_states:
        #     print(f" j1: {item[0]} j2: {item[1]}")

        compare_steps = len(joint_states) - 1
        len_joint_states = len(joint_states)
        for x in range(compare_steps):
            # print('\n')
            vel_j0 = diff_angle(joint_states[x, 0], joint_states[x + 1, 0], 'j1') / time_step
            vel_j1 = diff_angle(joint_states[x, 1], joint_states[x + 1, 1], 'j2') / time_step

            if vel_j0 > self.maxvel0 or vel_j1 > self.maxvel1:
                print(f"angular velocity limit exceeds for at step {x}")
                failed_vel_at_step[x] = {False: [joint_states[x, 0], joint_states[x, 1]]}

        if len(failed_vel_at_step):
            print("all failed velocity steps: \n")
            print(failed_vel_at_step)
            return False
        else:
            print("chk_joint_vel_satisfied(): succeeded, returning: True")
            return True

    # End of class R2Link


def find_point2point_trajectory(p0, p1, step_size):
    """
    This function finds path points to create a trajectory between 2 points with desired number of steps from point
    to point roboticstoolbox library have ctraj() method that finds trajectory in cartesian space along a straight line.
    :param p0:
    :param p1:
    :return: trajectory consists of path points
    """
    # dist in meters
    print("finding trajectory: ")
    dist = math.dist(p0, p1)
    # #step size in milimeters.
    step = dist * step_size
    print(f"dist: {dist}")
    T0 = SE3(p0)
    T1 = SE3(p1)
    # print("T0:")
    # print(T0)
    # print("T1:")
    # print(T1)

    # print("Ts: \n")
    Ts = roboticstoolbox.tools.trajectory.ctraj(T0, T1, int(step))
    # print(Ts)
    # sol = robot.ikine_LM(Ts)
    print("\n")
    return Ts


def find_solved_joint_states_multiple_points(work_path_points, robot_created):
    """
    This Function finds solved reachable joint states for all the points in a trajectory for a specified robot.
    this is a combination of two previously defined methods above find_point2point_trajectory() and
    solve_reachable_ikin() to find the total solution at a time for all the points in described goal points.

    :param work_path_points:
    :param robot_created:
    :return: all the solved path point reachable by robot
    """

    work_path_solved_joint_states = np.array([[0.0, 0.0, 0.0]])
    print(f"\nnumber of lines in work_path_points-> {len(work_path_points)} \n")

    line_count = 0
    for path in work_path_points:
        # print(path)
        line_count += 1
        p0 = np.array([path[0], path[1], path[2]])
        p1 = np.array([path[3], path[4], path[5]])
        print(f"from p0: {p0} to p1: {p1}")
        T_p0_p1 = find_point2point_trajectory(p0, p1, 10)
        print(f"number of steps in Trajectory: {len(T_p0_p1)}")
        solved_joint_states = robot_created.solve_reachable_ikin(T_p0_p1)

        if len(solved_joint_states) == 0:
            print(f"\n unreachable goal at line{line_count} for {p0} to {p1} \n")
            if path[1] == path[4] == 0.0:
                print(f"failure is caused by Zero(0.0) value due to mathematical reason...\n"
                      f"try again by slightly changing value to non-Zero value")
                p0_mod = np.array([path[0], path[1] + 0.000001, path[2]])
                p1_mod = np.array([path[3], path[4] + 0.000001, path[5]])

                T_p0_p1_mod = find_point2point_trajectory(p0_mod, p1_mod, 10)
                print(f"number of steps in Trajectory: {len(T_p0_p1_mod)}")
                solved_joint_states_mod = robot_created.solve_reachable_ikin(T_p0_p1_mod)
                if len(solved_joint_states_mod) == 0:
                    print(f"error again after modification: unreachable goal at line{line_count} for {p0} to {p1} \n")
                else:
                    work_path_solved_joint_states = np.concatenate(
                        (work_path_solved_joint_states, solved_joint_states_mod))
                    # print(solved_joint_states_mod)
                    print(f"solved for line: {line_count} from p0: {p0} to p1: {p1}")

        else:
            # print(solved_joint_states)
            work_path_solved_joint_states = np.concatenate((work_path_solved_joint_states, solved_joint_states))
        time.sleep(1)

    print(f"length of total work_path_solved_joint_states is {len(work_path_solved_joint_states)} \n")
    work_path_solved_joint_states = np.delete(work_path_solved_joint_states, 0, 0)
    # print(work_path_solved_joint_states)

    return work_path_solved_joint_states


def create_robot(length0, length1, min0, max0, maxvel0, min1, max1, maxvel1):
    """
        This function create a robot from the all the fields retrieved from the robot description file.
        The created robot is an instance of R2Link class which inherits DHRobot class to create a robot
        using DH parameters.
    ::param length0: length of first link in meter
        :param length1: length of second link in meter
        :param min0: min rotation angle of j0 in degree
        :param max0: max rotation angle of j0 in degree
        :param maxvel0: maximum angular velocity
        :param min1: min rotation angle of j1 in degree
        :param max1: max rotation angle of j1 in degree
        :param maxvel1: maximum rotation velocity
    :return: robot as an instance of R2Link class
    """
    print("values received at function: create_robot()")
    print(f"robot link0 length0: {length0}, min0: {min0}, max0: {max0},maxvel0: {maxvel0} ")
    print(f"robot link1 length0: {length1}, min1: {min1}, max1: {max1},maxvel1: {maxvel1} ")
    # robot_created = R2Link(length0,length1,min0,max0,maxvel0,min1,max1,maxvel1)

    # Reason for deviding length0 and length1 with 1000:
    #       R2Link inherits DHRobot class which uses unit 'meter' for link length of the robot
    robot_created = R2Link(length0=length0 / 1000, length1=length1 / 1000,
                           min0=round(math.radians(min0), 5), max0=round(math.radians(max0), 5),
                           maxvel0=round(math.radians(maxvel0), 5),
                           min1=round(math.radians(min1), 5), max1=round(math.radians(max1), 5),
                           maxvel1=round(math.radians(maxvel1), 5))

    print(robot_created)
    print("Robot has initial configuration qz:")
    print(robot_created.qz)
    return robot_created


def read_robot_description(filename='robot_description.txt', sep=' '):
    """
        This function reads the robot's description file and check for validity of data type
        and number of fields. If the description is valid then the fields are returned to be
        used to create a robot using DH parameters.
    :param filename:
    :param sep:
    :return: robot's description as an array
    """
    fields = None
    try:
        # nonlocal fields
        f = open(filename, "r")
        fields = f.readline().split(sep=sep)
        print(f"successful data read from :{filename}")
        # print(fields)
    except:
        print(f"failed data read from :{filename}")
        return None

    print(f"number of variables: {len(fields)}")
    if len(fields) == 8:
        print(f"robot description from ' {filename} ' are shown below: ")
        print(fields)
        # length0, length1, min0, max0, maxvel0, min1, max1, maxvel1 = fields
        try:
            fields_as_float = [float(x) for x in fields]
            print(f"fields_as_float: {fields_as_float}")
            return fields_as_float
        except:
            print("error: int or float expected...")
            return []
    else:
        print(f"error: 8 values expected in {filename}, found {len(fields)} \n {fields}")
        return []


def read_path_description(filename='path_point.txt', sep=' '):
    """
        This function reads the work path point description file and check for validity of data type
        and number of fields. If the description is valid then the fields are returned to be
        used to create trajectory to be executed by robot within it's workspace.

    :param filename:
    :param sep:
    :return: array
    """
    print("reading path description consists of points...")
    lines = None
    all_path_points = []

    try:
        # nonlocal fields
        f = open(filename, "r")
        lines = f.readlines()
        print(f"successful data read from :{filename}")
        print(f"number of lines: {len(lines)}")

    except:
        print(f"failed data read from :{filename}")
        return None
    line_count = 0
    for line in lines:
        fields = line.split(sep)
        print(fields)
        line_count += 1

        print(f"number of variables: {len(fields)}")
        if len(fields) == 6:
            # print(fields)
            # length0, length1, min0, max0, maxvel0, min1, max1, maxvel1 = fields
            try:
                fields_as_float = [float(x) for x in fields]
                print(f"fields_as_float: {fields_as_float} \n")
                all_path_points.append(fields_as_float)

            except:
                print("error: int or float expected...")
                return []
        else:
            print(
                f"error: 6 values expected in each line {filename}, found {len(fields)} in line{line_count} \n {fields}")
            return []

    return all_path_points


def execute_joint_poses(conn, poses, robot):
    """
        executes the poses in a loop with a desired speed. this shares a connection 'conn' with
        multiprocessing() pipe. receives updated joint position via pipe and sends the joint
        states via pipe and also plots the movements of the robot for a visual representation.
     """

    print("executing joint poses: ")

    start_time = time.time()

    for pose in poses:
        robot.plot(pose)
        robot._servo_j0.rotate(pose[0])
        robot._servo_j1.rotate(pose[1])
        conn.send(pose)
        time.sleep(0.1)
    x = np.array([], dtype=np.float64)
    conn.send(x)
    return


def print_curr_joint(conn):
    """
        this function shares a connection 'conn' with multiprocessing() pipe.
        receives updated joint position via pipe and prints the msg
    """

    start_time = time.time()
    x = np.array([], dtype=np.float64)
    while True:
        msg = conn.recv()
        # print(f"received message {msg}")
        interval = time.time() - start_time
        if np.array_equal(msg, x):
            print("> empty array received: (process ended)")
            break
        # change this value to change the time interval of printing
        if interval >= 0.1:
            print(f" d> 0.0, j0> {math.degrees(msg[0])}, j1> {math.degrees(msg[1])}, j2> 0.0")
            start_time = time.time()


def execute_with_multiprocess(solved_joint_states_T_ini_to_work_start, robot_created, speed_of_execution):
    """
        This function all the solved joint states for the robot.
        This method can be considered essentially sending all necessary values to a real robot
        for executing on real motors and to display the joint states of the robot while system
        is active.
        I decided to use multiprocess so that the 2 process of running the robot and printing
        the joint states can be decentralized. I used multiprocessing.Pipe() which is a connection
        with 2 ends. the parent_conn executes the robot's movements and sends the value to the
        child_conn to be printed. In a real robot some sensor can send the data over a network
        connection on a central control and observation system.

    :param solved_joint_states_T_ini_to_work_start:
    :param robot_created:
    :return:
    """

    parent_conn, child_conn = multiprocessing.Pipe()

    p1 = multiprocessing.Process(target=execute_joint_poses,
                                 args=(parent_conn, solved_joint_states_T_ini_to_work_start, robot_created))
    p2 = multiprocessing.Process(target=print_curr_joint, args=(child_conn,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

    return True


def main(argv):
    """
    receive arguments from command line for the name of 2 files
    :param argv: <robot file> and <path point file>
    :return:
    """
    robot_file = ''
    path_file = ''
    speed_of_execution = 1.0
    try:
        opts, args = getopt.getopt(argv, ["ifile=", "ofile="])
        print(f"args are: {opts} and {args}")
        if len(args) == 2:
            robot_file, path_file = args
    except getopt.GetoptError:
        print('test.py <robot_file> <path_file>')
        sys.exit(2)
    print(f'\nrobot_file file is: {robot_file} path_file file is: {path_file}\n')

    # robot_description is a varible to store the description from the robot_file
    robot_description = read_robot_description(robot_file)

    print(f"found valid description: \n{robot_description} \n ")
    length0, length1, min0, max0, maxvel0, min1, max1, maxvel1 = robot_description
    time.sleep(1)
    robot_created = create_robot(length0, length1, min0, max0, maxvel0, min1, max1, maxvel1)

    # building total trajectory in parts: initial position to start position
    # forward kinematics finds the position of the end-effector in cartesian space

    fk = robot_created.fkine(robot_created.qz2)
    print("fk:\n")
    print(fk.t)
    print(len(fk.t))

    p_ini = np.array(fk.t)
    # p0 = np.array([1.5, 1.0, 0.0])
    # p1 = np.array([1.5, 0.0, 0.0])
    p_work_start = []
    p_work_end = []

    # work_path_points is a variable to store the path description from path_file
    work_path_points = read_path_description(path_file)

    # this will store solved joint stats for all the points in the total trajectory of the work
    # excluding trajectory for initial-to-work_start and work_end-to-initial positions.
    solved_joint_states_work_path_points = []

    if work_path_points:

        p_work_start = np.array(work_path_points[0][:3])
        p_work_end = np.array(work_path_points[-1][3:6])
        print(f"p_work_start: {p_work_start}")
        print(f"p_work_end: {p_work_end}")
        solved_joint_states_work_path_points = find_solved_joint_states_multiple_points(work_path_points, robot_created)


    else:
        print(f" empty work_path_points...")

    print(f"p_work_start: {p_work_start}")
    print(f"p_work_end: {p_work_end}")

    T_ini_to_work_start = find_point2point_trajectory(p_ini, p_work_start, 10)
    # T_p0_p1 = find_point2point_trajectory(p0, p1, 10)
    T_work_end_to_ini = find_point2point_trajectory(p_work_end, p_ini, 10)

    solved_joint_states_T_ini_to_work_start = robot_created.solve_reachable_ikin(T_ini_to_work_start)
    solved_joint_states_T_work_end_to_ini = robot_created.solve_reachable_ikin(T_work_end_to_ini)

    # combine all the trajectory to create a final trajectory:
    """
        check if the velocity condition is satisfied for 3 major steps:
        step 1: from initial position to work_start position
        step 2: from work_start position to work_end position
        step 3: from work_end position to initial position
        
    """

    # step 1: from initial position to work_start position

    vel_satisfied_T_ini_to_work_start = False

    if len(solved_joint_states_T_ini_to_work_start) > 1:
        print(
            f"calling chk_joint_vel_satisfied: maxvel_j0: {robot_created.maxvel0}, maxvel_j1: {robot_created.maxvel1}")
        vel_satisfied_T_ini_to_work_start = robot_created.chk_joint_vel_satisfied(
            solved_joint_states_T_ini_to_work_start, speed_of_execution)
        print(f"joint vel_satisfied_T_ini_to_work_start for all steps: {vel_satisfied_T_ini_to_work_start}")
    print(vel_satisfied_T_ini_to_work_start)

    # step 2: from work_start position to work_end position

    vel_satisfied_work_path_points = False

    if len(solved_joint_states_work_path_points) > 1:
        print(
            f"calling chk_joint_vel_satisfied: maxvel_j0: {robot_created.maxvel0}, maxvel_j1: {robot_created.maxvel1}")
        vel_satisfied_work_path_points = robot_created.chk_joint_vel_satisfied(solved_joint_states_work_path_points, speed_of_execution)
        print(f"joint velocity satisfied for all steps: {vel_satisfied_work_path_points}")
    print(vel_satisfied_work_path_points)

    # step 3: from work_end position to initial position

    vel_satisfied_T_work_end_to_ini = False

    if len(solved_joint_states_T_work_end_to_ini) > 1:
        print(
            f"calling chk_joint_vel_satisfied: maxvel_j0: {robot_created.maxvel0}, maxvel_j1: {robot_created.maxvel1}")
        vel_satisfied_T_work_end_to_ini = robot_created.chk_joint_vel_satisfied(solved_joint_states_T_work_end_to_ini,
                                                                                speed_of_execution)
        print(f"joint vel_satisfied_T_work_end_to_ini for all steps: {vel_satisfied_T_work_end_to_ini}")
    print(vel_satisfied_T_work_end_to_ini)

    """
        if velocity condition is satisfied for all 3 steps of the trajectory within the limit of robot's 
        configuration, then the final execution of the movement starts from here.   
        
        step 1: from initial position to work_start position
        step 2: from work_start position to work_end position
        step 3: from work_end position to initial position
    """

    # step 1: from initial position to work_start position

    print("movement execution will start in 3 seconds... ")
    time.sleep(3)

    if vel_satisfied_T_ini_to_work_start and vel_satisfied_work_path_points and vel_satisfied_T_work_end_to_ini:
        # step 1: from initial position to work_start position

        execution = execute_with_multiprocess(solved_joint_states_T_ini_to_work_start, robot_created, speed_of_execution)
        print(f" robot moved to work_start position: {execution}")
        # step 2: from work_start position to work_end position

        execution1 = execute_with_multiprocess(solved_joint_states_work_path_points, robot_created, speed_of_execution)
        print(f" robot moved to work_start position: {execution1}")

        # step 3: from work_end position to initial position
        execution2 = execute_with_multiprocess(solved_joint_states_T_work_end_to_ini, robot_created, speed_of_execution)
        print(f" robot moved to init position: {execution2}")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # main()
    main(sys.argv[1:])

# See PyCharm help at https://www.jetbrains.com/help/pycharm/