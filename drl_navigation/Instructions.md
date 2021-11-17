# DRL mapless navigation: DDPG
## Prerequisites:

1. Ubuntu 20.04
2. ROS noetic
3. Python 3.8.2
4. Pytorch 1.9.0
5. rtabmap-ros

## Installation

#####  1.  Install Ubuntu 20.04
#####  2. Install ROS noetic following the instructions on the [website](http://wiki.ros.org/noetic/Installation/Ubuntu)
#####  3. Install pytorch V.1.9:
```
$ pip3 install torch==1.9.0 torchvision==0.10.0  -f https://download.pytorch.org/whl/cpu/torch_stable.html
```
##### 4. Install [rtabmap-ros](http://wiki.ros.org/rtabmap_ros) . We use "rtabmap" to get the odometry topic of our Jetbot model in simulation
```
$ sudo apt install ros-noetic-rtabmap-ros
```
## Run the code
### Hyperparameters tuning:
####  1. Number of Lidar readings
##### Turtlebot
* Go to the "turtlebot3_burger.gazebo.xacro" file in turtlebot3/turtlebot3_description/urdf
* Change the number of samples (10 currently) for the lidar, along with the min_angle and max_angle. 
##### Jetbot
* Go to the "environment_jet_sim_mapless.py" file in the drl_navigation workspace
* Change the choice of Lidar measurements in the getState function
* Change the "min_range" parameters in the getState function based on the lidar being used
####  2. DRL hyperparameters
* In any of the ddpg algorithms python files ("ddpg_real_mapless.py","" ddpg_sim_mapless.py", etc.):
  * You can change the BATCH_SIZE, LEARNING_RATE, GAMMA and TAU
  * You can decide to train or load a trained model with the "is_training" variable
  * You choose where to save or load  your models from, by storing the folder name in the "world" variable. The path would be "dirPath + '/Models/' + world"
  * To load a model, use the function:  "trainer.load_models(2050) " in this case loads the model trained for 2050 episodes" 
* Update the reward function by going to the "environment_jet_sim_mapless.py" file in the drl_navigation workspace, and change the "setReward" function 
### Turtlebot robot 
#####  1. Download required ROS packages in the catkin workspace
```
$  cd ~/catkin_ws/src/
$  git clone 
$  cd ~/catkin_ws && catkin_make
```
#####  2. DRL navigation simulation- Mapless
* There are 3 stages to be considered: 1 (no obstacles), 2 (static obstacles), 3 (static and dynamic obstacles)
```
$  export SVGA_VGPU10=0 (in case you are running on a VM)
$  export TURTLEBOT3_MODEL=burger (you can change the "burger" model to the "waffle" one)
$  roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
* Go to "ddpg_sim_mapless.py", and change 
```
from environment_jet_sim_mapless import Env
```
to 
```
from environment_turtle_sim_mapless import Env
```
* Run the training
```
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}sim_mapless.launch
```

### Jetbot robot
####  1. Download required ROS packages in the catkin workspace
```
$  cd ~/catkin_ws/src/
$  git clone 
$  cd ~/catkin_ws && catkin_make
```
#### 2. DRL navigation simulation- Mapless (Please refer to the hyperparameter tuning section)
```
$  roslaunch jetbot_diff_drive jetbot_rviz_gazebo_{number_of_stage}.launch
$  roslaunch rtabmap_ros rtabmap.launch       use_sim_time:=false    depth:=false    subscribe_scan:=true    frame_id:=base_link    scan_topic:=/scan    icp_odometry:=true
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}sim_mapless.launch
```
#### 3. DRL navigation - Mapless  - Real robot
* On the robot (Master):
```
$  roslaunch rplidar_ros rplidar_s1.launch
$  rosrun jetbot_ros jetson_motors.py
```
* On the PC:
```
$  roslaunch hector_slam_launch tutorial.launch 
$  rostopic pub findObjects custom_msgs/Object -r 1 -- -0.5 0.4 0.0
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}_mapless.launch
```
#### 4. ROS navigation stack - Real robot
* On the robot (Master):
```
$  roslaunch rplidar_ros rplidar_s1.launch
$  rosrun jetbot_ros jetson_motors.py
```
* On the PC:
```
$  roslaunch classical_navigation tf_launch.launch
$  roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
$  roslaunch classical_navigation nav_launch.launch
```
#### 5. DDPG with AMCL - Simulation 
* Open the robot model in rviz and gazebo:
```
$  roslaunch jetbot_diff_drive jetbot_rviz_gazebo_{number_of_stage}.launch
```
* Create a map of our simulation
```
$  roslaunch jetbot_diff_drive jetbot_rviz_gazebo_{number_of_stage}.launch
$  roslaunch hector_slam_launch tutorial.launch 
$  rosrun map_server map_saver –f sim_env
```
* Save the map and move it to the maps folder: robot/src/drl_navigation/maps
* Run the DRL training on the saved map (navigation with AMCL)
```
$  roslaunch jetbot_diff_drive jetbot_rviz_gazebo_{number_of_stage}.launch
$  roslaunch drl_navigation tf_launch.launch
$  roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
$  roslaunch drl_navigation amcl_launch.launch
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}sim.launch
```
#### 6. DDPG with AMCL - Real robot
* On the robot (Master):
```
$  roslaunch rplidar_ros rplidar_s1.launch
$  rosrun jetbot_ros jetson_motors.py
```
* On the PC:
- Create a map of the environment
```
$  roslaunch drl_navigation tf_launch_real.launch
$  roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
$  roslaunch drl_navigation amcl_launch.launch
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}.launch
$  rostopic pub findObjects custom_msgs/Object -r 1 -- -0.5 0.4 0.0

```
## Next steps and WIP:
* Fix the "DDPG with AMCL - Real robot" section, by getting the pose estimate for AMCL every few iterations
* Find the most adequate parameters for the ROS classical navigation
* Upload the computer vision part of the system and update the instructions