import vrep

# Starts simulation in V-REP
def start_simulation():
    # You must have V-REP open and allowed to accept incoming connections for this to work
	# Close all open connections (Clear bad cache)
	vrep.simxFinish(-1)
	# Connect to V-REP (raise exception on failure)
	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	if clientID == -1:
		raise Exception('Failed connecting to remote API server')

	print("Starting simulation with clientID: ", clientID)
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
	return clientID

# Ends V-REP simulation
def end_simulation(clientID):
	# Stop simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)
	# Close the connection to V-REP
	vrep.simxFinish(clientID)
	print("End of simulation")
