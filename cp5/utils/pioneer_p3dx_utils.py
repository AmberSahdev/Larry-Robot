import vrep
import time
import numpy as np
import utils.utils as utils

# Checks the existence of pioneer_p3dx mobile platform in the .ttt scene
def check_pioneer_p3dx_exists(clientID):
    result, pioneerHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_connection9', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('Could not get Pioneer_p3dx handle for base frame')

# Returns the location of pioneer_p3dx in world coordinates
def get_pioneer_p3dx_location(clientID):
    result, pioneerHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_connection9', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, pioneerHandle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, pioneerHandle, floorHandle, vrep.simx_opmode_streaming)

    return position

# Moves pioneer_p3dx along 1 dimension given velocity and time
# Does so by moving both motors at the same rate for the same time
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

# Hardcoded velocity to move pioneer_p3dx to the left
def move_towards_target(clientID, alpha):
    # alpha : Optimal distance between UR3 and target object
    while utils.distance_to_target(clientID) > alpha:
        move_pioneer_p3dx(clientID, velocity = -(utils.distance_to_target(clientID) - alpha))
