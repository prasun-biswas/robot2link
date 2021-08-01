# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import math
import time
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH,backends
from roboticstoolbox import *
from spatialmath import SE3
from spatialmath import base



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
            RevoluteDH(d=base,
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

    def solve_reachable_ikin(self,Ts):
        #check if all the path points are reachable by the robot
        sol = self.ikine_LM(Ts)
        print("ikine Solution: \n")
        # print(sol)
        set_a = set(sol.success)
        print(set_a)
        if False in set_a:
            print("unreachable goal:")

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

    def execute_move(self, joint_states):

        self.plot_movement(joint_states)
        self.plot_movement(joint_states)
        self.plot_movement(joint_states)

        pass

    # End of class R2Link


class servo_motor():
    pass




def find_trajectory(p0, p1):
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
    step = dist*10
    print(f"dist: {dist}")
    T0 = SE3(p0)
    T1 = SE3(p1)
    print("T0: \n")
    print(T0)
    print("T1: \n")
    print(T1)

    print("Ts: \n")
    Ts = roboticstoolbox.tools.trajectory.ctraj(T0, T1,int(step))
    # print(Ts)
    # sol = robot.ikine_LM(Ts)
    return Ts


def create_robot():
    robot = R2Link()
    print(robot)
    print("configuration qz:")
    print(robot.qz2)
    return robot



def main():
    robot = create_robot()
    # declare 2 points for creating path

    p0 = np.array([1.5, 1.0, 0.0])
    p1 = np.array([1.5, 0.0, 0.0])
    Ts = find_trajectory(p0,p1)
    # print(Ts)
    solved_joint_states = robot.solve_reachable_ikin(Ts)
    print("solved joint states: ")
    # print(solved_joint_states)
    robot.plot_movement(solved_joint_states)
    # robot.execute_move(solved_joint_states)



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

    # robot = R2Link()
    # print(robot)
    # print("configuration qz:")
    # print(robot.qzn)
    # robot.plot(robot.qz)
    # time.sleep(1)
    # robot.plot(robot.qz1)
    # time.sleep(1)
    # robot.plot(robot.qz2).hold()

    # fk = robot.fkine([1.5708, 0,0])
    # print("fk:\n")
    # print(fk)
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

    # traj = jtraj(robot.qz, robot.qzn, 100)
    # traj.q.shape
    # print(traj.q)
    # t = numpy.arange(0,1,0.01)
    # T0 = SE3(1.5, 1.0, 0.0)
    # T1 = SE3(1.5, 0.0, 0.0)
    # Ts = roboticstoolbox.tools.trajectory.ctraj(T0, T1,10)
    # print(Ts)
    #
    # sol = robot.ikine_LM(Ts)
    # print(sol)
    # plt = qplot(traj.q)
    # display = robot.plot(robot.qz)
    #
    #
    # display.hold()

    # display.step()
    pass





# See PyCharm help at https://www.jetbrains.com/help/pycharm/
