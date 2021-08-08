# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import math
import sched, time
import numpy as np
from roboticstoolbox import *
from spatialmath import SE3
from collections import OrderedDict
from spatialmath import base
import multiprocessing, threading
import motor



class R2Link(DHRobot):
    def __init__(self,symbolic = False):
        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0
        deg = pi/180
        base = 0

        L =[
            RevoluteDH(d = base,
                       a = 1.5,
                       alpha=0.0,
                       offset=0.0,
                       qlim=None,
                       flip=False),
            RevoluteDH(d = base,
                       a = 1,
                       alpha=0.0,
                       offset=0.0,
                       qlim=None,
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
        self.addconfiguration("qz",[0,0,0])
        self.addconfiguration("qz1",[1,1,0])
        self.addconfiguration("qz2",[1.5708,-1.5708,0.0])
        self._servo_j1 = motor.servo_motor(0,'j1')
        self._servo_j2 = motor.servo_motor(0,'j2')


    def solve_reachable_ikin(self,Ts):
        #check if all the path points are reachable by the robot
        sol = self.ikine_LM(Ts)

        print("ikine Solution: \n")
        # print(sol)
        set_a = set(sol.success)
        print(set_a)
        if False in set_a:
            count_false = np.count_nonzero(sol.success ==False)
            itemindex = np.where(sol.success == False)
            print(f"unreachable goal:{count_false} at {itemindex[0]}")
            if count_false ==1 and itemindex[0]==0:
                print("false alarm at first position")
                q_modified = np.delete(sol.q, 0, 0)
                print("(q_modified) all pathpoints are reachable: \n")
                # print(q_modified)
                return q_modified

        else:
            print("all pathpoints are reachable")
            return sol.q

        return None

    def plot_movement(self, joint_states):

        """
        this plots the movements in 3d space
        :param joint_states:
        :return:
        """

        print("received joint states to show: \n")

        # print(joint_states)

        # combine begin, task, end poses

        for item in joint_states:
            # print(item)
            self.plot(item)
            time.sleep(0.001)
        # self.plot(joint_states[-1]).hold()

        pass


    @staticmethod
    def chk_joint_vel_satisfied(joint_states,max_vel_j1,max_vel_j2,speed:float):
        time_step = 1/speed
        failed_vel_at_step = OrderedDict()

        print("checking joint vel condition: \n")

        def diff_angle(x, y,joint='joint'):
            PI = math.pi
            angle = min((2 * PI) - abs(x - y), abs(x - y))
            # print(f"angle diff of {joint}: rad {angle} or deg: {np.rad2deg(angle)}")
            return angle

        # print("printing joint states for j1 and j2:  \n")
        # for item in joint_states:
        #     print(f" j1: {item[0]} j2: {item[1]}")

        compare_steps = len(joint_states)-1
        len_joint_states = len(joint_states)
        for x in range(compare_steps):
            # print('\n')
            vel_j1 = diff_angle(joint_states[x,0],joint_states[x+1,0],'j1')/time_step
            vel_j2 = diff_angle(joint_states[x,1], joint_states[x+1,1],'j2')/time_step

            if vel_j1>max_vel_j1 or vel_j2>max_vel_j2:
                print(f"angular velocity limit exceeds for at step {x}")
                failed_vel_at_step[x]={False:[joint_states[x,0], joint_states[x,1]]}

        print("all failed velocity steps: \n")
        print(failed_vel_at_step)
        if len(failed_vel_at_step):
            return False
        else:
            return True


    # End of class R2Link


def find_point2point_trajectory(p0, p1, step_size):
    """
    finds pathpoints to create a trajectory between 2 points
    :param p0:
    :param p1:
    :return:
    """
    #dist in meters
    print("finding trajectory: ")
    print(p0)
    dist = math.dist(p0,p1)
    # #step size in milimeters.
    step = dist*step_size
    print(f"dist: {dist}")
    T0 = SE3(p0)
    T1 = SE3(p1)
    print("T0: \n")
    print(T0)
    print("T1: \n")
    print(T1)

    # print("Ts: \n")
    Ts = roboticstoolbox.tools.trajectory.ctraj(T0, T1,int(step))
    # print(Ts)
    # sol = robot.ikine_LM(Ts)
    print("\n")
    return Ts


def create_robot():
    robot = R2Link()
    print(robot)
    print("configuration qz:")
    print(robot.qz2)
    return robot

# experiment block
def rotate_j1(robot, pose):
    robot._servo_j1.rotate(pose)

def rotate_j2(robot, pose):
    robot._servo_j2.rotate(pose)

def execute_joint_poses(conn,poses,robot):
    """ executes the poses in loop.
        this shares a connection 'conn' with multiprocessing() pipe.
            receives updated joint position via pipe and prints the msg
            """

    print("executing joint poses: ")

    start_time = time.time()

    for pose in poses:
        robot._servo_j1.rotate(pose[0])
        robot._servo_j2.rotate(pose[1])
        conn.send(pose)
        time.sleep(0.01)
    x = np.array([], dtype=np.float64)
    conn.send(x)
    return


def print_curr_joint(conn):
    """this shares a connection 'conn' with multiprocessing() pipe.
        receives updated joint position via pipe and prints the msg
        """

    start_time = time.time()
    x = np.array([], dtype=np.float64)
    while True:
        msg = conn.recv()
        # print(f"received message {msg}")
        interval = time.time() - start_time
        if interval >= 0.1:
            print(f"received message {msg}")
            start_time = time.time()
        if np.array_equal(msg,x):
            print("empty array received")
            break


    pass


#experiment block


def main():
    robot = create_robot()
    # declare 2 points for creating path

    # building total trajectory in parts: initial position to start position
    #                               start position to end position
    #                               end position to initial position
    fk = robot.fkine(robot.qz2)

    print("fk:\n")
    print(fk.t)
    print(len(fk.t))

    p_ini = np.array(fk.t)
    p0 = np.array([1.5, 1.0, 0.0])
    p1 = np.array([1.5, 0.0, 0.0])

    T_pini_p0 = find_point2point_trajectory(p_ini,p0,10)
    T_p0_p1 = find_point2point_trajectory(p0, p1, 100)
    T_p1_pini = find_point2point_trajectory(p1,p_ini, 10)

    # print(T_pini_p0)
    # print(T_p0_p1)
    # print(T_p1_pini)

    # solved_joint_states_T_pini_p0 = robot.solve_reachable_ikin(T_pini_p0)
    # print("solved_joint_states_T_pini_p0: \n ")
    # print(solved_joint_states_T_pini_p0)
    #
    solved_joint_states = robot.solve_reachable_ikin(T_p0_p1)
    print("solved_joint_states: \n ")
    print(solved_joint_states)

    # solved_joint_states_T_p1_pini = robot.solve_reachable_ikin(T_p1_pini)
    # print("solved_joint_states_T_p1_pini: \n")
    # print(solved_joint_states_T_p1_pini)

    # combine all the trajectory to create a final trajectory:
    vel_satisfied = False
    if len(solved_joint_states)>1:
        vel_satisfied = R2Link.chk_joint_vel_satisfied(solved_joint_states, 1.0, 1.0, 1.0)
        print(f"joint velocity satisfied for all steps: {vel_satisfied}")




    # if vel_satisfied:
    #     robot.execute_move(solved_joint_states)
    # robot.plot_movement(solved_joint_states)
    # robot.execute_move(solved_joint_states)


    #this is working to create pipe between two process
    if vel_satisfied:
        parent_conn, child_conn = multiprocessing.Pipe()

        p1 = multiprocessing.Process(target=execute_joint_poses, args=(parent_conn,solved_joint_states,robot))
        p2 = multiprocessing.Process(target=print_curr_joint, args=(child_conn,))

        p1.start()
        p2.start()

        p1.join()
        p2.join()

        print(p1.is_alive())





# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

    # robot = R2Link()
    # print(robot)
    # print("configuration qz2:")
    # robot.plot(robot.qz2)
    # time.sleep(1)
    # robot.plot(robot.qz1)
    # time.sleep(1)
    # robot.plot(robot.qz2).hold()

    # fk = robot.fkine([1.5708, 0,0])
    # fk = robot.fkine(robot.qz2)
    #
    # print("fk:\n")
    # print(fk.t)
    # print(type(fk))

    # ik = robot.ikine_LM(fk)
    # print("ik:\n")
    # print(ik)
    # q = ik[0]
    # print("q: \n")
    # print(q)
    # robot.plot(q).hold()

    # traj = jtraj(robot.qz, robot.qzn, 100)
    # traj.q.shape
    # print(traj.q)

    # traj = jtraj(robot.qz, robot.qz2, 100)
    # traj.q.shape
    # print(traj.q)
    # t = np.arange(0,1,0.01)
    # T0 = SE3(1.5, 1.0, 0.0)
    # T1 = SE3(1.5, 0.0, 0.0)
    # Ts = roboticstoolbox.tools.trajectory.ctraj(T0, T1,10)
    # print(Ts)
    # #
    # sol = robot.ikine_LM(Ts)
    # print(sol)
    # plt = qplot(traj.q)
    # display = robot.plot(robot.qz)
    #
    # display.hold()

    # display.step()
    pass





# See PyCharm help at https://www.jetbrains.com/help/pycharm/
