# import math
# import sys, getopt, time
# import numpy as np
# from roboticstoolbox import *
# from spatialmath import SE3
# from collections import OrderedDict
# import motor
#
# class R2Link(DHRobot):
#     """
#         This solution is built with roboticstoolbox library.
#
#         R2Link class is written by inheriting DHRobot class from roboticstoolbox library.
#         Class for robots defined using Denavit-Hartenberg notation
#         using DHRobot class robot is configured. This library allows to build robot with
#         joint angle rotation limitation. Also, this library calculates forward and inverse
#         kinematics for defined robot configuration.
#
#     """
#
#     def __init__(self, symbolic=False, length0=1.5, length1=1.0, min0=-90.0, max0=90.0, maxvel0=1.0, min1=-120,
#                  max1=120, maxvel1=1.0):
#         """
#
#         :param length0: length of first link in meter
#         :param length1: length of second link in meter
#         :param min0: min rotation angle of j0 in degree
#         :param max0: max rotation angle of j0 in degree
#         :param maxvel0: maximum angular velocity
#         :param min1: min rotation angle of j1 in degree
#         :param max1: max rotation angle of j1 in degree
#         :param maxvel1: maximum rotation velocity
#         """
#
#         self.length0 = length0
#         self.length1 = length1
#         self.min0 = min0
#         self.max0 = max0
#         self.maxvel0 = maxvel0
#         self.min1 = min1
#         self.max1 = max1
#         self.maxvel1 = maxvel1
#
#         print(f"\nvalues for R2Link robot initilization: \n {self.length0}, {self.length1},"
#               f" {self.min0},{self.max0}, {self.maxvel0}, "
#               f"{self.min1}, {self.max1}, {self.maxvel1}")
#
#         if symbolic:
#             import spatialmath.base.symbolic as sym
#             zero = sym.zero()
#             pi = sym.pi()
#         else:
#             from math import pi
#             zero = 0.0
#         deg = pi / 180
#         base = 0
#
#         L = [
#             RevoluteDH(d=base,
#                        a=self.length0,
#                        alpha=0.0,
#                        offset=0.0,
#                        qlim=[self.min0, self.max0],
#                        flip=False),
#             RevoluteDH(d=base,
#                        a=self.length1,
#                        alpha=0.0,
#                        offset=0.0,
#                        qlim=[self.min1, self.max1],
#                        flip=False),
#             RevoluteDH(d=0,
#                        a=0.0,
#                        alpha=0.0,
#                        offset=0.0,
#                        qlim=None,
#                        flip=False)
#         ]
#
#         super().__init__(L,
#                          name="Robot 2 link",
#                          keywords=('planar',)
#                          )
#         # self.addconfiguration("qz",[1.5708,-1.5708])
#         # the configuration creates different poses for robot using joint angle.
#         # for this solution default starting pose is qz2: [1.5708, -1.5708, 0.0]
#         self.addconfiguration("qz", [0, 0, 0])
#         self.addconfiguration("qz1", [1, 1, 0])
#         self.addconfiguration("qz2", [1.5708, -1.5708, 0.0])
#         self._servo_j0 = motor.servo_motor(0, 'j0')
#         self._servo_j1 = motor.servo_motor(0, 'j1')
#
#     def solve_reachable_ikin(self, Ts):
#         """
#             this method checks if all the Transformation matrix is reachable by the
#             configured robot. inverse kinematics to find the valid poses are calculated
#             by using ikine_LM method of DHRobot class inherited is used to solve for
#             reachable targets. Ikine_LM uses Numerical inverse kinematics by
#             Levenberg-Marquadt optimization(Robot superclass)
#
#         :param Ts: trajectory for a point to point movements
#         :return: solved joint states if all points are reachable
#         """
#         sol = self.ikine_LM(Ts)
#
#         print("ikine Solution: \n")
#         set_a = set(sol.success)
#         print(set_a)
#         if False in set_a:
#             count_false = np.count_nonzero(sol.success == False)
#             itemindex = np.where(sol.success == False)
#             print(f"unreachable goal:{count_false} at {itemindex[0]}")
#             if count_false == 1 and itemindex[0] == 0:
#                 # sometimes there are zero values at the first position of array which results alse
#                 # f warning of unreachable goal. As a bug fix the value is deleted from numpy array
#
#                 print("false warning at first position")
#                 q_modified = np.delete(sol.q, 0, 0)
#                 print("(q_modified) all pathpoints are reachable: \n")
#                 return q_modified
#
#         else:
#             print("all pathpoints are reachable")
#             # sol.q has all the solved valid poses. other un-necessary values are ignored
#             return sol.q
#         # returns empty if inverse kinematics  fails
#         return []
#
#     def plot_movement(self, joint_states):
#         """
#         this plots the movements of the configured robot in 3d space
#         :param joint_states: all the joint states the robot need to execute
#         :return:
#         """
#         print("received joint states to show: \n")
#         for item in joint_states:
#             # print(item)
#             self.plot(item)
#             time.sleep(0.001)
#         self.plot(joint_states[-1]).hold()
#
#     def chk_joint_vel_satisfied(self, joint_states, speed: float = 1.0):
#         """
#             This method checks if the robot's angular velocity limitation allows the robot to execute all the
#             movements along trajectory for all joints along all the points at a certain speed. The process is
#             finds the angular difference between 2 positions and the speed for each steps.
#
#         :param joint_states:
#         :param speed:
#         :return:
#         """
#         time_step = 1 / speed
#         failed_vel_at_step = OrderedDict()
#
#         print(f"\nchecking joint vel condition: maxvel_j0: {self.maxvel0}, maxvel_j1: {self.maxvel1} \n  ")
#
#         def diff_angle(x, y, joint='joint'):
#             PI = math.pi
#             angle = min((2 * PI) - abs(x - y), abs(x - y))
#             # print(f"angle diff of {joint}: rad {angle} or deg: {np.rad2deg(angle)}")
#             return angle
#
#         # print("printing joint states for j1 and j2:  \n")
#         # for item in joint_states:
#         #     print(f" j1: {item[0]} j2: {item[1]}")
#
#         compare_steps = len(joint_states) - 1
#         len_joint_states = len(joint_states)
#         for x in range(compare_steps):
#             # print('\n')
#             vel_j0 = diff_angle(joint_states[x, 0], joint_states[x + 1, 0], 'j1') / time_step
#             vel_j1 = diff_angle(joint_states[x, 1], joint_states[x + 1, 1], 'j2') / time_step
#
#             if vel_j0 > self.maxvel0 or vel_j1 > self.maxvel1:
#                 print(f"angular velocity limit exceeds for at step {x}")
#                 failed_vel_at_step[x] = {False: [joint_states[x, 0], joint_states[x, 1]]}
#
#         if len(failed_vel_at_step):
#             print("all failed velocity steps: \n")
#             print(failed_vel_at_step)
#             return False
#         else:
#             print("chk_joint_vel_satisfied(): succeeded, returning: True")
#             return True
#
#     # End of class R2Link
#
