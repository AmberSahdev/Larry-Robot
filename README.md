# Larry-Robot
ECE 470 Final Project

For our final project, we intend to build a simulation of a robot to which a user can point out unknown objects to. The user can then point out a second location and the robot then picks the object up and drops it off at the new location.

We intend to use computer vision techniques to determine the position of the object that the user points at, inverse kinematics to locomote towards the object, reinforcement learning to teach our robot how to pick up previously unknown objects and potentially learn the physics of picking objects up, and forward kinematics to move towards a dynamically determined destination (by pointing at a general area). 

We have decided to use V-REP for simulation purposes. We will also consider using UR3 within the simulator, with added capabilities, in hopes to physically actuate that robot before the end of the semester. 

The robot will have six degrees of motion including either legs or wheels, whichever we deem appropriate during the project. We intend to use multiple cameras to determine the x,y,z coordinates of our dynamic pick-up and drop-off locations. 
