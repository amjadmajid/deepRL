# Reinforcement Learning with DDPG

This DDPG-DRL node aims to provide DRL navigation towards coordinates "x" and "y" received through the topic: /findObjects

## Libraries

[Pytorch]

## ROS 
cd ~/catkin_ws/src/
git clone {link_git}
cd ~/catkin_ws && catkin_make

## Run Code
I have four stage as in the examples of Robotis. But I dont know yet my code dont have a geat performance in stage 3.

First to run:
```
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
In another terminal run:
```
roslaunch project ddpg_stage_{number_of_stage}.launch
```