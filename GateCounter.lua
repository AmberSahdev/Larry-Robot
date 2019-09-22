function sysCall_init() 
    counter=0
    previousCounter=-1
    previousDetectionState=0
    sensorHandle=sim.getObjectHandle("CountingGateSensor_80cmX190cm")
    displayHandle=simGetUIHandle("CountingGateDisplay_80cmX190cm")
    gateFullName=sim.getObjectName(sim.getObjectAssociatedWithScript(sim.handle_self))
    simSetUIButtonLabel(displayHandle,0,gateFullName)
    activeCol={1.0,0.1,0.1}
    passiveCol={0.1,0.1,0.1}
    delay=0.2
    v=tonumber(sim.getScriptSimulationParameter(sim.handle_self,'nonDetectionDelayForNextTriggerActivation'))
    if (v~=nil) then
        if (v<0) then v=0 end
        delay=v
    end
    lastDetection=-10
end

function sysCall_cleanup() 
 
end 

function sysCall_sensing() 
    st=sim.getSimulationTime()
    detectionState=sim.readProximitySensor(sensorHandle)
    if (detectionState==1) then
        if ((st-lastDetection>=delay)and(previousDetectionState==0)) then
            counter=counter+1
        end
        lastDetection=st
    end
    previousDetectionState=detectionState
    
    if (previousCounter~=counter) then
        c=math.fmod(counter,1000)
        for i=0,2,1 do
            d=math.floor(c/(10^(2-i)))
            b=100+i*10
            if (d==0) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,passiveCol)
                simSetUIButtonColor(displayHandle,b+4,activeCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==1) then
                simSetUIButtonColor(displayHandle,b+0,passiveCol)
                simSetUIButtonColor(displayHandle,b+1,passiveCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,passiveCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,passiveCol)
            end
            if (d==2) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,passiveCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,activeCol)
                simSetUIButtonColor(displayHandle,b+5,passiveCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==3) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,passiveCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==4) then
                simSetUIButtonColor(displayHandle,b+0,passiveCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,passiveCol)
            end
            if (d==5) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,passiveCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==6) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,passiveCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,activeCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==7) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,passiveCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,passiveCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,passiveCol)
            end
            if (d==8) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,activeCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            if (d==9) then
                simSetUIButtonColor(displayHandle,b+0,activeCol)
                simSetUIButtonColor(displayHandle,b+1,activeCol)
                simSetUIButtonColor(displayHandle,b+2,activeCol)
                simSetUIButtonColor(displayHandle,b+3,activeCol)
                simSetUIButtonColor(displayHandle,b+4,passiveCol)
                simSetUIButtonColor(displayHandle,b+5,activeCol)
                simSetUIButtonColor(displayHandle,b+6,activeCol)
            end
            c=c-d*(10^(2-i))
        end
    end
    previousCounter=counter
end 
