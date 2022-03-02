# DRL mapless navigation: DDPG
## Prerequisites:

1. Ubuntu 20.04
2. ROS noetic
3. Python 3.8.2
4. Pytorch 1.9.0

## Installation

#####  1.  Install Ubuntu 20.04
#####  2. Install ROS noetic following the instructions on the [website](http://wiki.ros.org/noetic/Installation/Ubuntu)
#####  3. Install pytorch V.1.9:
```
$ pip3 install torch==1.9.0 torchvision==0.10.0  -f https://download.pytorch.org/whl/cpu/torch_stable.html 
```
#####  4. Install monoloco:
```
$ pip3 install monoloco

```
### Running the Turtlebot robot simulation in the environment
#####  1. Download required ROS packages in the catkin workspace
```
$  cd ~/catkin_ws/src/
$  git clone 
$  cd ~/catkin_ws && catkin_make
```
#####  2. Edit the simulation
Edit the world file: 
```
$  catkin_ws/src/mybot_gazebo/worlds/mybot.world
```
#####  3. Run the simulation
```
$  export TURTLEBOT3_MODEL=waffle
$  roslaunch mybot_gazebo breach_detection.launch
```
### Running the Hybrid navigation
#####  1. Run the YOLO 3D detection node
```
$  roslaunch darknet_ros_3d darknet_ros_3d.launch
```
#####  2. Run the hybrid detection node
```
$  roslaunch sort_track sort.launch
```
