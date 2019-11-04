import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm
import utils.math_utils as math_utils
import utils.ur3_utils as ur3_utils
import utils.vrep_utils as vrep_utils

def main():
    clientID = vrep_utils.start_simulation()
    time.sleep(2)

    ur3_utils.check_UR3_exists(clientID)
    [base_handle, joint_one_handle, joint_two_handle, joint_three_handle, joint_four_handle, joint_five_handle, joint_six_handle] = ur3_utils.get_joint_handles(clientID)

    time.sleep(2)
    vrep_utils.end_simulation(clientID)
    return

if __name__ == '__main__':
	main()
