import vrep
import time
import numpy as np
from numpy import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils
import modern_robotics as mr

# Dynamic function to get the current M and S of the robot
def get_current_MS():
    return

# Get M of robot in its zero configuration
def get_zeroconfig_M():
    return

# Get S of robot in its zero configuration
def get_zeroconfig_S():
    return

# Get T transformation matrix of the object to be picked up
# This is the end-effector position we are trying to reach
def get_dest_T():
    return


def main():
    clientID = vrep_utils.start_simulation()
    time.sleep(2)

    ur3_utils.check_UR3_exists(clientID)
    [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = ur3_utils.get_joint_handles(clientID)
    jointHandles = [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle]
    print("> V-REP and UR-3 now setup")

    # Setup/Test logic
    # Set robot to all angles 0
    ur3_utils.set_joint_position(np.array([0, 0, 0, 0, 0, 0]), clientID, jointHandles)
    print("> UR3 now in zero configuration")
    # define x, y, z on vrep plane
    # analyze robot configuration to find M
    # analyze robot configuration to find S

    # IK logic
    print("> Inverse Kinematics")
    T_1in0 = get_dest_T()
    M = get_zeroconfig_M()
    S = get_zeroconfig_S()

    initThetas = [0, 0, 0, 0, 0, 0, 0] # initial thetas robot starts at
    # error parameters
    eomg = [0.0001]
    ev = [0.0001]

    [targetThetaList, IKsuccess] = mr.IKinSpace(S, M, T_1in0, initThetas, eomg, ev)

    if IKsuccess:
        print("\t Target joint angles: ", targetThetaList)
        ur3_utils.set_joint_position(np.array(targetThetaList), clientID, jointHandles)
    else:
        print("\t Target position unreachable either due to current method or due to physical constraints")


    print("> Exiting Simulation")
    time.sleep(2)
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
