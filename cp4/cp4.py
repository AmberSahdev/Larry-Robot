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

# Get M of robot in its zero configuration
def get_zeroconfig_M(clientID, jointHandles):
    ur3_utils.set_zero_config(clientID, jointHandles)
    jointDistances = ur3_utils.get_joint_distances(clientID, jointHandles)
    p = [jointDistances[0][-1], jointDistances[1][-1], jointDistances[2][-1]]
    R = np.eye(3) # no rotation
    M = np.zeros((4,4))
    M[:3, :3] = R
    M[:3, 3] = p
    M[3, 3] = 1

    return M

# Get S of robot in its zero configuration
def get_zeroconfig_S(clientID, jointHandles):
    # analyze robot configuration to find S
    ur3_utils.set_zero_config(clientID, jointHandles)
    jointDistances = ur3_utils.get_joint_distances(clientID, jointHandles)

    w1 = np.array([0, 0, 1])
    q1 = [jointDistances[0][0], jointDistances[1][0], jointDistances[2][0]]
    v1 = np.cross(-w1, q1)
    S1 = np.append(w1, v1).reshape((6, 1))

    w2 = np.array([-1, 0, 0])
    q2 = [jointDistances[0][1], jointDistances[1][1], jointDistances[2][1]]
    v2 = np.cross(-w2, q2)
    S2 = np.append(w2, v2).reshape((6, 1))

    w3 = np.array([-1, 0, 0])
    q3 = [jointDistances[0][2], jointDistances[1][2], jointDistances[2][2]]
    v3 = np.cross(-w3, q3)
    S3 = np.append(w3, v3).reshape((6, 1))

    w4 = np.array([-1, 0, 0])
    q4 = [jointDistances[0][3], jointDistances[1][3], jointDistances[2][3]]
    v4 = np.cross(-w4, q4)
    S4 = np.append(w4, v4).reshape((6, 1))

    w5 = np.array([0, 0, 1])
    q5 = [jointDistances[0][4], jointDistances[1][4], jointDistances[2][4]]
    v5 = np.cross(-w5, q5)
    S5 = np.append(w5, v5).reshape((6, 1))

    w6 = np.array([-1, 0, 0])
    q6 = [jointDistances[0][5], jointDistances[1][5], jointDistances[2][5]]
    v6 = np.cross(-w6, q6)
    S6 = np.append(w6, v6).reshape((6, 1))

    S = np.append(S1, S2, axis=1)
    S = np.append(S, S3, axis=1)
    S = np.append(S, S4, axis=1)
    S = np.append(S, S5, axis=1)
    S = np.append(S, S6, axis=1)

    return S

# Get T transformation matrix of the object to be picked up
# This is the end-effector position we are trying to reach
def get_dest_T(clientID, jointHandles):
    # Dummy function for now
    # list of points that work:
    # 1. [0.1, 1.2, 0.1, 0.1, 0, 0]
    # 2. [0.1, -0.6, 0.1, 0.1, 0, 0]
    # 3. [0.1, 0.6, 0.1, 0.1, 0, 0]
    # 4. [0, PI/2, -0.5, 0, 0, 0]

    # print("Dest")
    ur3_utils.set_joint_position(np.array([0, 1.2, -0.2, -0.7, 0, 0]), clientID, jointHandles)
    print("\t Showing target position to reach later in V-REP")
    jointDistances = ur3_utils.get_joint_distances(clientID, jointHandles)

    T = np.eye(4)
    R = np.eye(3)
    p = np.array([jointDistances[0][-1], jointDistances[1][-1], jointDistances[2][-1]])
    T[:3, :3] = R
    T[:3, 3] = p
    T[3, 3] = 1

    print("\t Dest T: ", T)

    # TODO: one contraint to ease would be to develop a method agnostic of rotation
    # useful for end-effectors that dont rely on orientation as much, such as suction cups
    return T

# Write Test Functions
def test_IK():
    # Test 1: point reachable by UR3
    # Test 2: point reachable by ur3 but not in the right rotation
    # Test 3: point unreachable physically
    return

def main():
    clientID = vrep_utils.start_simulation()
    time.sleep(2)

    ur3_utils.check_UR3_exists(clientID)
    [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = ur3_utils.get_joint_handles(clientID)
    jointHandles = [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle]
    print("> V-REP and UR-3 now setup")

    # x, y, z on vrep plane: V-REP has x, y, z markers on the bottom right of the screen

    # IK logic
    print("> Inverse Kinematics")
    M = get_zeroconfig_M(clientID, jointHandles)
    S = get_zeroconfig_S(clientID, jointHandles)
    T_1in0 = get_dest_T(clientID, jointHandles)

    time.sleep(5)

    ur3_utils.set_zero_config(clientID, jointHandles)
    time.sleep(2)
    print("\t UR3 now in zero configuration")
    initThetas = [0, PI/2, 0, 0, 0, 0] # initial thetas robot starts at (PI/2 because most objects are going to be on the ground)
    eomg = [0.2]
    ev = [0.2] # error parameters
    [targetThetaList, IKsuccess] = mr.IKinSpace(S, M, T_1in0, initThetas, eomg, ev)

    if IKsuccess:
        print("\t Target joint angles: ", targetThetaList)
        # Move to the calculated position
        ur3_utils.set_joint_position(np.array(targetThetaList), clientID, jointHandles)
    else:
        print("\t Target position unreachable either due to current algorithm or due to physical constraints")

    time.sleep(5)

    print("> Exiting Simulation")
    time.sleep(2)
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
