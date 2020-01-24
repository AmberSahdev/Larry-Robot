# Demonstrates pick and place on UR3 using suction cup
import vrep
import time
import numpy as np
from numpy import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils

PI = np.pi

def main():
    clientID = vrep_utils.start_simulation()
    ur3_utils.check_UR3_exists(clientID)
    jointHandles = ur3_utils.get_joint_handles(clientID)
    print("> V-REP and UR-3 now setup")

    vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active', 0, vrep.simx_opmode_oneshot)
    time.sleep(2)
    print("> Set UR-3")
    ur3_utils.set_zero_config(clientID, jointHandles)
    theta = np.array([0.001, 1.26, 0.2, 0.2, -PI/2, 0.001])
    ur3_utils.set_joint_position(theta, clientID, jointHandles)
    time.sleep(3)

    # suction cup
    vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active', 1, vrep.simx_opmode_oneshot)

    # move up
    theta = np.array([-1.2, -1.00, -0.4, -0.2, PI/2, 0.001])
    ur3_utils.set_joint_position(theta, clientID, jointHandles)
    time.sleep(2)
    vrep.simxSetIntegerSignal(clientID, 'BaxterVacuumCup_active', 0, vrep.simx_opmode_oneshot)
    time.sleep(2)

    print("> Exiting Simulation")
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
