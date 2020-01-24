import vrep
import time
import numpy as np
import modern_robotics as mr

timeBetweenJointMovements = 0.3
PI = np.pi

######################### Functions to initialize UR3 #########################
# Check existence of UR-3 in the frame/.ttt file
def check_UR3_exists(clientID):
	# Get "handle" to the base of robot
	# Setup "handle"
	result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for base frame')

# Returns a list of joint handles of UR3: 1 base handle and 6 joint handles
def get_joint_handles(clientID):
	# UR-3 joints
	# Get "handle" to the base of robot
	result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for base frame')

	# Get "handle" to the all joints of robot
	result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for first joint')
	result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for second joint')
	result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for third joint')
	result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for fourth joint')
	result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for fifth joint')
	result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for sixth joint')

	return [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle]

# TODO: returns the handle for end-effector on the arm of our UR3
def get_end_effector_handle(clientID):
	# TODO: get end-effector working
	# Get "handle" to the end-effector of robot
	result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('Could not get UR3 handle for end effector')
	return end_handle


####################### Functions to read UR3 metadata #######################
# Function to get the location of the UR3 in the world frame
def get_ur3_location(clientID):
    result, UR3Handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
    result, floorHandle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_blocking)

    result, position = vrep.simxGetObjectPosition(clientID, UR3Handle, floorHandle, vrep.simx_opmode_streaming)
    while position == [0, 0, 0]:
        result, position = vrep.simxGetObjectPosition(clientID, UR3Handle, floorHandle, vrep.simx_opmode_streaming)

    return position

# Function used to move joints to desired angle theta
def set_joint_position(theta, clientID, jointHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = jointHandles

	'''
	for jointHandle in jointHandles[1:]:
		vrep.simxSetJointTargetVelocity(clientID, jointHandle, 0.01, vrep.simx_opmode_oneshot)
	'''

	for jointHandle in jointHandles[1:]:
		vrep.simxSetJointForce(clientID, jointHandle, 27, vrep.simx_opmode_oneshot)


	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(timeBetweenJointMovements)

# Returns distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint_distances(clientID, jointHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = jointHandles

	X = []
	Y = []
	Z = []
	result,vector=vrep.simxGetObjectPosition(clientID, joint_one_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_two_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_three_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_four_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_five_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=vrep.simxGetObjectPosition(clientID, joint_six_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])

	# TODO: after adding end-effector functionality
	"""
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	"""
	X = np.round(X, decimals = 5)
	Y = np.round(Y, decimals = 5)
	Z = np.round(Z, decimals = 5)
	return X,Y,Z

# Returns list of current joint angles of UR3
def get_joint_angle(clientID, jointHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = jointHandles

	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	thetas = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return thetas

'''
Given a target transformation matrix, finds the set of angles to reach that T
After finding the target angles, sets the robot joints to those angles
Returns if inverse kinematic solution was found and the calculated target angles
'''
def UR3_inverse_kinematics(targetT, clientID, jointHandles):
	# x, y, z on vrep plane: V-REP has x, y, z markers on the bottom right of the screen
	# TODO: one contraint to ease would be to develop a method agnostic of rotation
	# above idea useful for end-effectors that dont rely on orientation as much, such as suction cups
	M = get_zeroconfig_M(clientID, jointHandles)
	S = get_zeroconfig_S(clientID, jointHandles)

	initThetas = [0, PI/2, 0, 0, 0, 0] # initial thetas robot starts at (PI/2 because most objects are going to be on the ground)
	eomg = [0.2]; ev = [0.2] # error parameters
	# IKinSpace() uses the Newton-Rahpson method
	[targetThetaList, IKsuccess] = mr.IKinSpace(S, M, targetT, initThetas, eomg, ev)

	if IKsuccess:
	    set_joint_position(np.array(targetThetaList), clientID, jointHandles)
	# Reasons for not being succesful: either due to the method or due to physical constraints
	return IKsuccess, targetThetaList

# Turns the suction cup attached to the UR3 on or off 
def suction(control, clientID):
    # control = 0 == off
    # control = 1 == on
    vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active', control, vrep.simx_opmode_oneshot)


########################## Helper Functions for UR3 ##########################
# Sets the UR-3 to all zero angles/zero configuration
def set_zero_config(clientID, jointHandles):
    set_joint_position(np.array([0, 0, 0, 0, 0, 0]), clientID, jointHandles)

# Get M of robot in its zero configuration
# Can be precomputed and made return a static value
def get_zeroconfig_M(clientID, jointHandles):
    set_zero_config(clientID, jointHandles)
    jointDistances = get_joint_distances(clientID, jointHandles)
    p = [jointDistances[0][-1], jointDistances[1][-1], jointDistances[2][-1]]
    R = np.eye(3) # no rotation
    M = np.zeros((4,4))
    M[:3, :3] = R
    M[:3, 3] = p
    M[3, 3] = 1

    return M

# Get S of robot in its zero configuration
# Can be precomputed and made return a static value
def get_zeroconfig_S(clientID, jointHandles):
    # analyze robot configuration to find S
    set_zero_config(clientID, jointHandles)
    jointDistances = get_joint_distances(clientID, jointHandles)

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
