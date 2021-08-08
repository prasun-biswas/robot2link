import numpy as np
import math
class servo_motor():

    def __init__(self, start_pos: int,joint):
        self._start_pos = start_pos
        self._curr_pos = start_pos
        self._joint_name = joint
        print(f'motor ready: {joint}' )


    def rotate(self,new_pos):
        self._curr_pos = new_pos
        print(f"curr pose {self._joint_name}: {self._curr_pos}")
        return True
    @property
    def get_curr_pose(self):
        return self._curr_pos



def diff_angle(x,y):
    PI = math.pi
    angle = min((2 * PI) - abs(x - y), abs(x - y))
    print(f"angle diff is: rad {angle} or deg: {np.rad2deg(angle)}")


# def main():
#     servo_j1 = servo_motor(0,'j1')
#     state_reached = servo_j1.rotate(2)
#     print(state_reached)


    # diff_angle(0, 1.570796326814532)
    # a = np.arange(12).reshape(4, 3)
    # print(f"length: {len(a)}")
    # print(a)
    # print("printing difference: \n")
    # for x in range(len(a)-1):
    #     diff_1 = a[x,0]-a[x+1,0]
    #     print(diff_1)


    pass

if __name__ == '__main__':
    main()


