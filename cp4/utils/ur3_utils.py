import vrep
import time

timeBetweenJointMovements = 0.2

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
# Function used to move joints to desired angle theta
def SetJointPosition(theta, clientID, listHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = listHandles

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
def get_joint(clientID, listHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = listHandles

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
	result,vector=vrep.simxGetObjectPosition(clientID, end_handle,base_handle,vrep.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Returns list of current joint angles of UR3
def GetJointAngle(clientID, listHandles):
	[base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = listHandles

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
