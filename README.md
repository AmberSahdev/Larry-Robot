# Larry-Robot
ECE 470 Final Project

![Robot](/images/overview.png)

### Checkpont 1
For our final project, we intend to build a simulation of a robot to which a user can point out unknown objects to. The user can then point out a second location and the robot then picks the object up and drops it off at the new location.

We intend to use computer vision techniques to determine the position of the object that the user points at, inverse kinematics to locomote towards the object, reinforcement learning to teach our robot how to pick up previously unknown objects and potentially learn the physics of picking objects up, and forward kinematics to move towards a dynamically determined destination (by pointing at a general area).

We have decided to use V-REP for simulation purposes. We will also consider using UR3 within the simulator, with added capabilities, in hopes to physically actuate that robot before the end of the semester.

The robot will have six degrees of motion including either legs or wheels, whichever we deem appropriate during the project. We intend to use multiple cameras to determine the x,y,z coordinates of our dynamic pick-up and drop-off locations.

### Checkpoint 2
Video:  [Checkpoint 2](https://youtu.be/yc8lZDzVkz0)

At first, navigating V-REP was difficult because of the GUI and lack of quality tutorials available online. It was also difficult to get the remote API working for MacOS and Windows. However, we were able to learn LUA and write scripts to make a Robotnik Summit-XL move in a desired way. We added multiple sensor measurements such as gate counters and proximity sensors to our simulation environment. Every time our robot passed the gate counter it would increment the count, and every time the proximity sensor detected an obstacle it would stop.

Based on the information we gathered during this checkpoint, our project still seems feasible. We will need to fix our remote API access for faster prototyping and debugging. We will also need to start thinking of, and reading research papers on, methods for designing our reinforcement learning algorithm to teach our robot how to pick stuff up.

### Checkpoint 3
Video:  [Checkpoint 3](https://youtu.be/MoAWaUwMhmc)

To run the code in this checkpoint open V-REP and load in the ur3.ttt scene in cp3. Make sure V-REP can accept incoming connections and then type the following command in the terminal in project/cp3/
```
python3 cp3.py
```
The relevant remote API connection files are already in the cp3 directory.
The code takes our UR-3 to angles listed in the Goal_joint_angles variable in the main function.

We also looked into some research projects such as [Visual Pushing and Grasping](https://github.com/andyzeng/visual-pushing-grasping) by Andy Zeng to learn common practices, setup, and look for information. We derived inspiration to do that from the video [This Robot Learned To Clean Up Clutter](https://www.youtube.com/watch?v=txHQoYKaSUk)

We hope to get inverse kinematics and basic camera sensing working for the next checkpoint.

### Checkpoint 4
Video:  [Checkpoint 4](https://youtu.be/nPya1e8HnMI)

To run the code in this checkpoint open V-REP and load in the ur3.ttt scene in /cp4/. Make sure V-REP can accept incoming connections and then type the following command in the terminal in project/cp4/
```
python3 cp4.py
```
The relevant remote API connection files are already in the cp4 directory.

Our method calculates the M and S matrices of the robot and then the transformation matrix T of the desired end effector position to reach the object. Having these 3 parameters with an error bound, the algorithm from IKinSpace from HW 8's modern robotics library calculates whether or not this configuration is reachable and if it is then what angles satisfy the requirement.

We also restructured our code into ur3_utils, vrep_utils, and math_utils to make interaction with the robot and the environment easier and much easier to collaborate on.  

We hope to get perception and end-effector working by the next checkpoint.


### Checkpoint 5
At this point we have a robotic arm on a mobile base that, given a target, locomotes towards it, picks it up, and places the target object on a second specified position. The videos can be seen in the /videos/ directory and document our progress very well. Our choice of end-effector was a suction cup, our mobile base has two wheels.

To run the code in this checkpoint open V-REP and load in the ur3.ttt scene in /cp5/. Make sure V-REP can accept incoming connections and then type the following command in the terminal in project/cp5/
```
python3 cp5.py
```


Start                          |          Pick Up              |          Drop Off
:-------------------------:|:------------------------:|:-------------------------:
![1](/images/overview.png) | ![2](/images/pickingup.png) | ![3](/images/droppingoff.png)


### Future Features
- [ ] Determining object location through computer vision
- [ ] Path Planning
- [ ] Collision Detection and Avoidance

