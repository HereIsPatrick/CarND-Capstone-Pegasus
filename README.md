![alt text](./imgs/self-driving-car-engineer-nanodegree--nd013.jpg)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

| Pegasus member  | email | 
|:-----------------|:-------|
| Patrick Chiu |hereispatrick@gmail.com|
| Radhesh Bhat || 
| Gary Holness ||


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
YOLO ROS: Real-Time Object Detection for ROS is [here](https://github.com/leggedrobotics/darknet_ros/)
After clone darknet ros, change configuration of our darknet ros node, subscribe to /image_color.
Second change configuration of TLDetector node to subscribe boundingboxes
``string Class 
	float64 probability 
	int64 xmin 
	int64 ymin 
	int64 xmax 
	int64 ymax 
``


##### Traffic Light Detection Node
Subscribe image_color and boundingboxes topic for Traffic Light Detection Node. 

We only process traffic light class, and probaility greater than 0.85 in simulator mode. 0.25 in site mode.

Check size of boundingbox, diagonal of boundingbox grater than 80 pixel in simulator mode. 40 pixel in site mode. we will handle it.

In simulator, we have waypoints, to calculate traffic light waypoint index and state.


##### Traffic Light classifier
We have two procedure for simulator and site mode.

In simulator mode, we convert image to hsv color space, filter by Red, Yello and Green color.

- Red : 0~20, 340~360 degree.
- Yello : 40~70 degree.
- Green : 90~140 degree.

![alt text](./imgs/hsl_top.jpg)
'''

       # Step. convert to hsv space.
        hsv_bb_img = cv2.cvtColor(bb_image, cv2.COLOR_BGR2HSV)

        # Reference from https://stackoverflow.com/questions/42882498/what-are-the-ranges-to-recognize-different-colors-in-rgb-space
        # Red color
        hsv_red1_range = cv2.inRange(hsv_bb_img, (0 / 360 * 255, 70, 50), (20/360 * 255, 255, 255))
        hsv_red2_range = cv2.inRange(hsv_bb_img, (340.0 / 360 * 255, 70, 50), (360.0/360 * 255, 255, 255))

        # Yellow color
        hsv_yellow_range = cv2.inRange(hsv_bb_img, (40.0 / 360 * 255, 100, 100), (70.0 / 360 * 255, 255, 255))

        # Green color
        hsv_green_range = cv2.inRange(hsv_bb_img, (90.0 / 360 * 255, 100, 100), (140.0 / 360 * 255, 255, 255))
        
'''

We count it after filter, if count pixel greater than 50 pixel in the color.
we can make sure the light state.

'''

        PIXEL_THRESHOLD = 50
        # Red range in hsv space that two region ,(0~20) and (340~360)degree.
        if (cv2.countNonZero(hsv_red1_range) + cv2.countNonZero(hsv_red2_range))/2 > PIXEL_THRESHOLD:
            print('Red Light')
            return TrafficLight.RED
        elif cv2.countNonZero(hsv_yellow_range) > PIXEL_THRESHOLD:
            print('Yellow Light')
            return TrafficLight.YELLOW
        elif cv2.countNonZero(hsv_green_range) > PIXEL_THRESHOLD:
            print('Green Light')
            return TrafficLight.GREEN
        else:
            print('Unknow')
            return TrafficLight.UNKNOWN

'''




![alt text](./imgs/tl_classifie_simulator_full.png)

In site mode, 

1. Partition traffic light image to 3 regions.
2. Convert it to grayscale image.
3. Filter it under grey threshold 210 to 255
4. Count pixel of each region
5. Get the light state base on maxium counting pixel of region. 

We can see below right image. maxium counting pixel of region is green light partition.

![alt text](./imgs/tl_classifie_real_full.png)

## Planning Subsystem
##### Subject 1
##### Subject 2
##### Subject 3

## Control Subsystem
##### Subject 1
##### Subject 2
##### Subject 3

## Result of Video
#### Simulator(Highway Time-lapse with music)
[![Simulator(Highway Time-lapse with music)](http://img.youtube.com/vi/fiB9qJFm0lE/0.jpg)](https://youtu.be/fiB9qJFm0lE
 "Simulator(Highway Time-lapse with music)")
 
#### Simulator(Highway full version)
[![Simulator(Highway full version)](http://img.youtube.com/vi/tdR3GoSJLqA/0.jpg)](https://youtu.be/tdR3GoSJLqA
 "Simulator(Highway full version)") 
 
#### Simulator(Churchlot)
[![Simulator(Churchlot)](http://img.youtube.com/vi/4XgFrIKWlvM/0.jpg)](https://youtu.be/4XgFrIKWlvM
 "Simulator(Churchlot)")

#### Churchlot traffic light training
[![Churchlot traffic light training](http://img.youtube.com/vi/mDkHKSULjNw/0.jpg)](https://youtu.be/mDkHKSULjNw
 "Churchlot traffic light training")


