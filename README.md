# Larry-Robot

![Robot](/images/overview.png)

Larry is a simulation robot useful for highly general-purpose pick and place tasks. Users can point out unknown objects and a destination location to Larry and it picks the object up and drops it off at the destination.

Larry is a UR3 arm within a V-REP simulation. Larry's end effector is a suction cup, over other options such as claws, so as to make our picking and placing as general as possible regardless of size of target object. Larry is also placed on a mobile platform so that it can locomote towards far away target objects and destinations.

Larry can be seen in action in [simple_demo.mov](https://youtu.be/ji6-GIDG7Ro).

To run the code in this checkpoint open V-REP and load in the ur3.ttt scene. Make sure V-REP can accept incoming connections and then type the following command in the terminal
```
python3 demo.py
```
The relevant remote API connection files are already in the remoteAPI directory. Larry has the relevant forward and inverse kinematics as well as end effector functions in utils/ur3_utils.py. The mobile platform helper functions are in utils/pioneer_p3dx_utils.py.




Start                          |          Pick Up              |          Drop Off
:-------------------------:|:------------------------:|:-------------------------:
![1](/images/overview.png) | ![2](/images/pickingup.png) | ![3](/images/droppingoff.png)


### Future Features
- [ ] Determining object location through computer vision
- [ ] Path Planning
- [ ] Collision Detection and Avoidance
