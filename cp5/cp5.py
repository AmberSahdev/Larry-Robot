import vrep
import time
import numpy as np
from numpy import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils
import utils.pioneer_p3dx_utils as pioneer_p3dx_utils
import modern_robotics as mr
import utils.utils as utils

PI = np.pi

def get_target_T(clientID):
    p = utils.get_cuboid_location(clientID)
    print("\t Position of block: ", p)
    R = np.eye(3)
    T = np.zeros((4,4))
    T[:3, :3] = R
    T[:3, 3] = p
    T[3, 3] = 1
    return T

def main():
    clientID = vrep_utils.start_simulation()
    ur3_utils.check_UR3_exists(clientID)
    jointHandles = ur3_utils.get_joint_handles(clientID)
    pioneer_p3dx_utils.check_pioneer_p3dx_exists(clientID)
    print("> V-REP, UR-3, Pioneer_p3dx now setup")

    alpha = 0.59 # Optimal distance between UR3 and target object

    ur3_utils.set_zero_config(clientID, jointHandles)
    print("> Moving towards target")
    pioneer_p3dx_utils.move_towards_target(clientID, alpha)

    print("> Inverse Kinematics")
    targetT = get_target_T(clientID)
    IKsuccess, calculatedThetaList = ur3_utils.UR3_inverse_kinematics(targetT, clientID, jointHandles)
    ur3_utils.suction(1, clientID)
    if not IKsuccess:
        print("\t Target position unreachable either due to current algorithm or due to physical constraints")
    time.sleep(2)

    print("> Forward Kinematics")
    destThetas = [PI/2, -1.2, -0.7, 0, PI/2, 0]
    ur3_utils.set_joint_position(destThetas, clientID, jointHandles)
    ur3_utils.suction(0, clientID)
    ur3_utils.set_zero_config(clientID, jointHandles)

    print("> Exiting Simulation")
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
