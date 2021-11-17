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
pip3 install torch==1.9.0 torchvision==0.10.0  -f https://download.pytorch.org/whl/cpu/torch_stable.html
```
## Run the code
### Hyperparameters tuning:
####  1. Number of Lidar readings
* Go to the "turtlebot3_burger.gazebo.xacro" file in turtlebot3/turtlebot3_description/urdf
* Change the number of samples (10 currently) for the lidar, along with the min_angle and max_angle. 

####  2. DRL hyperparameters
* In  `ddpg_sim_mapless.py` you can:
  * change the BATCH_SIZE, LEARNING_RATE, GAMMA and TAU
  * decide to train or load a trained model with the "is_training" variable
  * choose where to save or load  your models from, by storing the folder name in the `world` variable. The path would be "dirPath + '/Models/' + world"
  * To load a model, use the function:  "trainer.load_models(2050) " in this case loads the model trained for 2050 episodes" 
* Update the reward function by going to the "environment_jet_sim_mapless.py" file in the drl_navigation workspace, and change the "setReward" function 
#####  1. Download this repo
```
$  cd ~/catkin_ws/src/
$  git clone -b sim_drl_nav https://github.com/amjadmajid/deepRL.git
$  cd ~/catkin_ws && catkin_make
```
#####  2. DRL navigation simulation- Mapless
* There are 3 stages to be considered: 1 (no obstacles), 2 (static obstacles), 3 (static and dynamic obstacles)
```
export SVGA_VGPU10=0 (in case you are running on a VM)
export TURTLEBOT3_MODEL=burger (you can change the "burger" model to the "waffle" one)
roslaunch turtlebot3_gazebo turtlebot3_stage_{number_of_stage}.launch
```
* Run the training
```
$  roslaunch drl_navigation ddpg_stage_{number_of_stage}sim_mapless.launch
```
