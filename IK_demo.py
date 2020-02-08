# Demonstrates Inverse Kinematics on UR3
import vrep
import time
import numpy as np
from numpy import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils

PI = np.pi

# Get T transformation matrix of the object to be picked up
# This is the end-effector position we are trying to reach
def get_target_T(clientID, jointHandles):
    # Dummy test function
    # [0, PI/2, -0.5, 0, 0, 0]
    T = np.array([[ 1. ,    0. ,    0.,    -0.112],
                  [ 0. ,    1. ,    0.,     0.506],
                  [ 0. ,    0. ,    1.,     0.251],
                  [ 0. ,    0. ,    0.,     1.   ]], dtype=np.float32)
    return T


def main():
    clientID = vrep_utils.start_simulation()
    ur3_utils.check_UR3_exists(clientID)
    jointHandles = ur3_utils.get_joint_handles(clientID)
    print("> V-REP and UR-3 now setup")

    print("> Target position")
    targetT = get_target_T(clientID, jointHandles)
    time.sleep(1)

    print("> Inverse Kinematics")
    IKsuccess, calculatedThetaList = ur3_utils.UR3_inverse_kinematics(targetT, clientID, jointHandles)
    if not IKsuccess:
        print("\t Target position unreachable either due to current algorithm or due to physical constraints")
    time.sleep(3)

    print("> Exiting Simulation")
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
