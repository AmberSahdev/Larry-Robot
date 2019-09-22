function sysCall_threadmain()
    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

        
    sim.setJointTargetVelocity(motorHandles[1],1)
    sim.setJointTargetVelocity(motorHandles[2],-1)
    sim.setJointTargetVelocity(motorHandles[3],-1)
    sim.setJointTargetVelocity(motorHandles[4],1)
end

