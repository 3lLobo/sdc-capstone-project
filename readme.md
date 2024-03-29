# Self-Driving Car - The Final Project

### Student: Florian Wolf

## About the project

This is the final Project of the Self-Driving Car Engineer Nanodegree. The goal is to drive a car around the test track autonomously. Therefore we will use waypoints to navigate the vehicle. The car should stop at red traffic lights and avoid obstacles. Each waypoint will have the map coordinates and a target velocity. The code is divided in a perception, a planning and a control subsystem.
We will use ROS (http://www.ros.org/) to implement the code on the vehicle. The code will be tested on the Udacity simulator first.


### Perception: Traffic Light Detection

To get the perception of a red traffic light, the traffic lights in our image need to be detected and then identified whether they are red or not. There for the view of the car's camera is essential, we subscribe to 'camera/image_raw'. It is also necessary to subscribe to 'base_waypoints', to get the base waypoints  and to 'current_pose' for the position of the car.
The 'sim_traffic_light_config.yaml' file provides the coordinates of all the traffic lights. With the current location of the car the nearest traffic light can be detected.
To identify the color of the light a Convolutional Neural Network (CNN) was build and trained. Here you can see the architecture of the CNN:


![alt text](/model.jpg "model")


To train a CNN a labeled data set is needed. Pictures of traffic lights were taken from the simulator and labeled as red or non red light. In order to enlarge the dataset and to make the model more robust. the images were zoomed and shifted.
after training the model and saving the weights, the CNN model was tested on a test image. It classified the red light correctly.
The Tensorflow Objet detection API was used to implement the model on the real world vehicle, the Udacity self-driving car Carla.


### Planing: The Waypoint Updater

The waypoint update predicts 50 waypoints and their speed. The update speed is 50 Hz. The coordinates and the speed of each waypoint are submitted to the `/final_waypoints` topic.
By default the car will constantly accelerate to the maximum speed. If a red light is detected, the speed of the waypoints will be decreased so that the car stops prior to the traffic light. If no red light has been detected, then the waypoint updater accelerate the car back to maximum speed. if the speed is already reduced the car will approach the traffic light slowly.


### Control: DBW

The Drive by WIre (DWB) node is located in the dwb_node.py file. It passes the speed and coordinated of the waypoint on to the controller object. The controller is located in the file twist_controler.py. It calculates the throttle, breaking and the steering. The controller works together with a PID and a lowpass. The PID controller the throttle and the brake. The lowpass gets the steering commands from the yaw controller and makes the steering smooth.


## Conclusion

This Project gave me a great understanding of ROS. It taught me how to combine all the different areas I have learned in this nanodegree course. With the tools given I managed to create a operating system with the three core attributes, Perception, Planning and Control. Weekpoints are the PID fine-tuning and the weights of the CNN. In real life the CNN should be trained with much more data. concerning the PID there should be different parameters for different situations.




### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download the model from:
https://drive.google.com/open?id=1SW2Li7qo0WI6erqbVg_Q4yufjdzXr2RL


2. Place the file in (sdc-capstone/ros/src/tl_detector).


3. Launch project in site mode:

cd CarND-Capstone/ros
```
roslaunch launch/site.launch
```






