import vrep
import time
import numpy as np
import utils.ur3_utils as ur3_utils

# returns the location of the cuboid in the world frame
def get_cuboid_location(clientID):
    result, cuboidObjHandle = vrep.simxGetObjectHandle(clientID, 'Cuboid', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, cuboidObjHandle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, cuboidObjHandle, floorHandle, vrep.simx_opmode_streaming)

    return np.array(position)

# returns distance between the cuboid to be picked up and the UR3 on top of the pioneer_p3dx
def distance_to_target(clientID):
    cuboidLocation = get_cuboid_location(clientID)
    UR3Location =  ur3_utils.get_ur3_location(clientID)
    distanceToTarget = np.linalg.norm(np.array(UR3Location[:2]) - np.array(cuboidLocation[:2]))
    return distanceToTarget
