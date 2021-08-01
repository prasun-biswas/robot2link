import numpy as np

import numpy as np
import math
import time


BP = brickpi3.BrickPi3()  # Create an instance of the BrickPi3 class

# TO DO. Set your number of joints here
nrOfJoints = 3
qNow = np.array([0.0] * nrOfJoints)  # This will be our joints state, global access in this script
# This int can be used to check wether there are pending requests to be attended by the planner or not
pendantRequest = 0
# TO DO. Student definitions, customize this with your data
# Specify ports you are using for your JOINTS in order, size must be the same than nrOfJoints
jointsOrder = [0, 1, 2]
# TO DO. Again, same size than your kinematic model in the planner or failure will be upon you, conversion from radians or mm to degrees of motors
jointsScale = np.array([3 * 180 / math.pi, 4 * 180 / math.pi, 36])  # This values are a random example


#
def callbackPlanner(data):
    global qNow
    global pendantRequest

    # TO DO. How many degrees of precision for each motor do you want?
    degreesThreshold = np.array([5, 5, 5, 5])  # You can also specify the same for every joint as done in kp and ki
    kp = [15] * len(data.points[0].positions)  # You can also specify a kp for each joint as done in degreesThreshold
    ki = [25] * len(data.points[0].positions)
    kd = [20] * len(data.points[
                        0].positions)  # kp ki and kd are proportional, integral and derivative constants for control respectively
    h = len(data.points)  # h contains the number of jointPoints
    w = len(data.points[0].positions)  # w contains number of joints
    Matrix = [[0 for x in range(w)] for y in range(h)]
    for idy, point in enumerate(data.points):
        # Flag for waiting until motors reach each point
        allReached = False
        # This array of flags indicates that each single joint arrived to the destination
        qReached = [False] * len(data.points[0].positions)
        timeBase = time.time()
        while not allReached:
            if rospy.is_shutdown():
                BP.reset_all()
                return
            for idx, jointPos in enumerate(point.positions):
                rospy.loginfo("Joint %s position to send %s", idx, jointPos)
                # Tell the motors to move to the desired position
                K = kp[idx] + ki[idx] * (time.time() - timeBase)
                if K > 100: K = 100
                BP.set_motor_position_kp(jointsOrder[idx],
                                         K)  # We can cheat the control and add an integrator varying kp over time
                BP.set_motor_position_kd(jointsOrder[idx], kd[idx])
                BP.set_motor_position(jointsOrder[idx], jointPos * jointsScale[idx])  # Note how jointsScale is used!
                for idx, jointPort in enumerate(jointsOrder):
                    if abs(BP.get_motor_encoder(jointPort) - int(
                            round(data.points[idy].positions[idx] * jointsScale[idx]))) < degreesThreshold[idx]:
                        qReached[idx] = True
                    if all(flag == True for flag in qReached):
                        allReached = True

    # Now the request has been attended, let's refresh qNow values
    for idx, q in enumerate(jointsOrder):
        # TO DO: Maybe it is better to put in this line below the last jointPoint given by the trajectory planner
        qNow[idx] = float(BP.get_motor_encoder(q)) / float(jointsScale[idx])
    # Request has been attended, success!
    pendantRequest = pendantRequest - 1
    rospy.loginfo("Request attended")






####################################################



# This is the main function
def task():

    # TO DO:
    # Request the planner to move our robot, you can start adding code here to make the first trials, but at the end, these movement requests
    # should be send in the callback listening matlab commands.
    # Be sure that the point is reachable, orientation can be ignored, as always, an example is given.

    ratePendant.sleep()  # Wait a while before requesting

    xyz = np.array([20.0, 20.0, 25.0])
    eulerzyx = np.array([0.0, 0.0, 0.0])  # Don't care about rotation now
    sendRequest('J', qNow, xyz, eulerzyx, 1)  # In theory one point for joint trajs is ok
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([20.0, 20.0, 15.0])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while not rospy.is_shutdown():
        ratePendant.sleep()
        rospy.loginfo("Pendant requests: %s", pendantRequest)


if __name__ == '__main__':
    try:
        task()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

