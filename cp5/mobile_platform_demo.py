import vrep
import time
import numpy as np
from numpy import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils
import modern_robotics as mr

PI = np.pi

def get_cuboid_location(clientID):
    result, cuboidObjHandle = vrep.simxGetObjectHandle(clientID, 'Cuboid', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, cuboidObjHandle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, cuboidObjHandle, floorHandle, vrep.simx_opmode_streaming)

    return position

def get_ur3_location(clientID):
    result, UR3Handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, UR3Handle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, UR3Handle, floorHandle, vrep.simx_opmode_streaming)

    return position

def get_pioneer_p3dx_location(clientID):
    result, pioneerHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_connection9', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, pioneerHandle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, pioneerHandle, floorHandle, vrep.simx_opmode_streaming)

    return position

def move_pioneer_p3dx(clientID, velocity=1, moveTime=1):
    result, motorLeft  = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_blocking)
    result, motorRight = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_blocking)

    if abs(velocity) < 0.9:
        velocity = 2*velocity

    vrep.simxSetJointTargetVelocity(clientID, motorLeft, velocity, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, motorRight, velocity, vrep.simx_opmode_oneshot)
    time.sleep(moveTime)
    vrep.simxSetJointTargetVelocity(clientID, motorLeft, 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, motorRight, 0, vrep.simx_opmode_oneshot)
    return

def distance_to_target(clientID):
    cuboidLocation = get_cuboid_location(clientID)
    UR3Location = get_ur3_location(clientID) # get_pioneer_p3dx_location(clientID)
    distanceToTarget = np.linalg.norm(np.array(UR3Location[:2]) - np.array(cuboidLocation[:2]))
    print('distanceToTarget: ', distanceToTarget)
    return distanceToTarget

def main():
    clientID = vrep_utils.start_simulation()
    # ur3_utils.check_UR3_exists(clientID)
    # jointHandles = ur3_utils.get_joint_handles(clientID)
    print("> V-REP and UR-3 now setup")

    print("> Init Pioneer_p3dx")

    alpha = 0.57 # Optimal distance between UR3 and target object

    while distance_to_target(clientID) > alpha:
        move_pioneer_p3dx(clientID, velocity = -(distance_to_target(clientID) - alpha))


    time.sleep(3)

    print("> Exiting Simulation")
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
