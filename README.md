![alt text](./imgs/self-driving-car-engineer-nanodegree--nd013.jpg)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

| Pegasus member  | email | 
|:-----------------|:-------|
| Patrick Chiu |hereispatrick@gmail.com|
| Radhesh Bhat || 
| Gary Holness |gary.holness@gmail.com|


### Environment
| Item  | Detail | 
|:-----------------|:-------|
| CPU |Intel i5 - 6 CPU|
| RAM |16G| 
| GPU | GTX 1060|
| OS | ubuntu 16.04|
| ROS |ROS Kinetic|
| CUDA | 9.0 |
| cuDNN | v7.2.1|

### Installation
##### Download via git
```bash
https://github.com/HereIsPatrick/CarND-Capstone-Pegasus.git
```

##### Install python dependence
```bash
cd CarND-Capstone-Pegasus
pip install -r requirements.txt
```

##### Download darknet_ros via git
```bash
cd ros/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```
##### Patch darknet_ros (subscription image_color)
``` bash
cd darknet_ros
patch -p1 ../darknet_ros_yolo.patch
cd ../../
```

##### Make and run simulator
```bash
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

##### Make and run site
```bash
catkin_make
source devel/setup.sh
roslaunch launch/site.launch
```


## System Architecture Diagram
The following is a system architecture diagram showing the ROS nodes and topics used in the project. 

![alt text](./imgs/final-project-ros-graph-v2.png)

## Perception Subsystem
This capstone project that use car's camera to percept traffic light, feedback light state(Red,Yellow and Green) and nearest waypoint to system.

##### Use YOLOv3 to recognize traffic light
We choose YOLOv3 to recognize traffic light, because of real-time performance is better, We can get a easy for use ros node. YOLO ROS is created and maintain by Marko Bjelonic(Robotic Systems Lab, ETH Zurich), Based on the COCO dataset, YOLO can detect the 80 COCO object classes. The pre-trained model of the convolutional neural network is able to detect pre-trained classes including traffic light.

##### Darknet ROS

##### Traffic Light Detection Node

##### Traffic Light classifier

## Planning Subsystem
##### Subject 1
##### Subject 2
##### Subject 3

## Control Subsystem
The Control Subsust consists of two major components.  These are the Waypoint Follower and
the Drive by Wire (DBW) components.  Our DBW implementation is found in the src/twist_controller
directory.  The DBW node consists of a PID controller, low pass filter, along with control 
logic presented as a ROS node, namely the DBW node.  Implementation focused on twist_controller.py
and dbw_node.py.   The DBW node subscribes to ROS topics

- /current_velocity
- /twist_cmd
- /vehicle/dbw_enables

and publishes to ros topics

- /vehicle/throttle_cmd
- /vehicle/steering_cmd
- /vehicle/brake_cmd

#####  dbw_node.py
The DBW node publishes steering, throttle, and brake commands at 50Hz.  This is the frequency at which the system on Carla (Udacity Autonomous Car) expects messages, otherwise autonomy will disengage if control messages are published at less than 10Hz.  As Carla is an automatic vehicle, if no braking or throttle is applied, the car will roll forward.   The breaking force employed is 700 Nm of torque.  

Team members who tested in the simulator on a slower machine running simulator natively and ROS in a VM on virtual box found that the rate needed to be slowed to 10Hz to contend with a lag issue due to communications of published topics from the simulator overwhelming ROS.  This issue manifested when the camera was turned on in the simulator.

##### twist_controller.py
PID control parameters kp,ki,kd were set to kp=8.5, ki=0.005, and kd= 6.0.  A long cycle of testing and selecting parameters was performed.  This was done by successive doubling followed by fine adjustment after observation of resulting driving profile.  Observation was made that the highway driving contained long stretches of curved roadway resulting in large accumulated error.  For this reason, ki was selected to be relatively small.  Observation was also made that the roadway had wavy segments that approach quickly.  Mitigating navication in these segments warranted responsive derivative component.  For this reason, kd was set relatively large.   Lastly, the highway included agressive curved sections of roadway.   Navigating through this warranted a proportional response capable of keeping car in lane around such curves.  Therefore, the choice for kp was set high relative to the derivative constant.  The church lot course involved a very sharp turn that looped around in a tight space.  Examining the waypoints, the curve was both sharp and the difference between successive waypoints was quickly changing.  Again, the approrpriate settings was a larger proportional component, followed by somewhat smaller derivative component, and a small integral component.  The setting we arrived upon satisfied these
observtions.   

The resulting control parameters were tested on a relatively slow machine (update rate 10Hz to deal with lag) both on the highway course and the church lot course.   For both, the resulting controller successfully navigated the tracks exhausting all of the waypoints. These results were reproducable.

The cutoff frequency was tested at a number of settings.  This began fairly agressive with tau=0.03, 0.05, 0.2, and 0.5.  This was done in conjnction with increasing the maximum throttle value to 0.8.  Given the increased throttle value, a less agressive cutoff frequency of tau=0.5 gave best results.  This makes sense becase as velocity increases, so to does the frequency or rate of change in velocities because you may find the need to want to change rapidly.

##### Dealing with lag

When running ROS on the Ubuntu 16.04 Virtual Machine and the simulator natively, there was a serious issue with lag.  The issue concerns the simulator publishing topics to the ROS nodes on the VM faster than can be handled.  After many hours of investigation, information online points to the ROS messaging protocol. This issue arose when the camera was turned on in the simulation (highway).  A real fix would involve the ability to slow down the simulator's publish rate of Image messages to /image_color topic.  In lieu of that, addressing it included slowing down dbw_node.py rate to 10Hz and turning on tcp_nodelay=True in subscriptions for TwistStamped messages for /twist_cmd and /current_velocity topics.  When testing on a slower machine, these helped mitigate the lag issue.  The lag issue caused the car to oscillate and become uncontrollable because the control commands were out of synch with the state of the simulator due to ROS on the VM being overwhelmed messages published by the simulator.
