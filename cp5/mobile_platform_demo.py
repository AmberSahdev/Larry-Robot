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

    return np.array(position)

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

    if abs(velocity) < 1.0:
        velocity = 3*velocity

    vrep.simxSetJointTargetVelocity(clientID, motorLeft, velocity, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, motorRight, velocity, vrep.simx_opmode_oneshot)
    time.sleep(moveTime)
    vrep.simxSetJointTargetVelocity(clientID, motorLeft, 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, motorRight, 0, vrep.simx_opmode_oneshot)
    return

def distance_to_target(clientID):
    cuboidLocation = get_cuboid_location(clientID)
    UR3Location =  get_ur3_location(clientID) # get_pioneer_p3dx_location(clientID) #
    distanceToTarget = np.linalg.norm(np.array(UR3Location[:2]) - np.array(cuboidLocation[:2]))
    print('distanceToTarget: ', distanceToTarget)
    return distanceToTarget

def check_pioneer_p3dx_exists(clientID):
    result, pioneerHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_connection9', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('Could not get Pioneer_p3dx handle for base frame')

def get_target_T(clientID):
    p = get_cuboid_location(clientID)
    print("Position of block: ", p)
    R = np.eye(3)
    T = np.zeros((4,4))
    T[:3, :3] = R
    T[:3, 3] = p
    T[3, 3] = 1
    return T

def suction(control, clientID):
    # control = 0 == off
    # control = 1 == on
    vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active', control, vrep.simx_opmode_oneshot)

def main():
    clientID = vrep_utils.start_simulation()
    ur3_utils.check_UR3_exists(clientID)
    jointHandles = ur3_utils.get_joint_handles(clientID)
    check_pioneer_p3dx_exists(clientID)
    print("> V-REP, UR-3, Pioneer_p3dx now setup")

    alpha = 0.59 # Optimal distance between UR3 and target object

    while distance_to_target(clientID) > alpha:
        move_pioneer_p3dx(clientID, velocity = -(distance_to_target(clientID) - alpha))

    # time.sleep(3)

    ur3_utils.set_zero_config(clientID, jointHandles)

    originalThetas = [0, 1.4, 0.45, 0, -PI/2, 0]
    ur3_utils.set_joint_position(originalThetas, clientID, jointHandles)
    suction(1, clientID)
    destThetas = [PI/2, -1.2, -0.7, 0, PI/2, 0]
    ur3_utils.set_joint_position(destThetas, clientID, jointHandles)
    suction(0, clientID)
    ur3_utils.set_zero_config(clientID, jointHandles)

    '''
    print("> Inverse Kinematics")
    targetT = get_target_T(clientID)
    IKsuccess, calculatedThetaList = ur3_utils.UR3_inverse_kinematics(targetT, clientID, jointHandles)
    if not IKsuccess:
        print("\t Target position unreachable either due to current algorithm or due to physical constraints")
    '''
    time.sleep(2)


    print("> Exiting Simulation")
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
