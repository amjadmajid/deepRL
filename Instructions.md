# DRL mapless navigation: DDPG
#### On the robot:
#####  1. Run in terminal 1:: 
```
$ roslaunch rplidar_ros rplidar_s1.launch
```
##### 2. Run in terminal 2 (set the max_pwm parameter  to 57 in "jetson_motor.py" ):
```
$ rosrun jetbot_ros jetson_motors.py
```
#### On the PC:
#####  1. Run: 
```
$ catkin_make
```
##### 2. Run in terminal 1:
```
$ roslaunch hector_slam_launch tutorial.launch
```
##### 3. Run in terminal 2  (change the coordinates according to your needs: x=-1 and y=0.15 in this case): 
```
$ rostopic pub findObjects custom_msgs/Object -r 1 -- -1.0 0.15 1.0
```
##### 4. Run in terminal 3::
```
$ rostopic echo /scanmatch_odom
```
##### 5. Run in terminal 4:
```
$ roslaunch DRL_nav ddpg_stage_1_mapless.launch
```
##### 6. Run in terminal 5 (to stop the robot)::
```
$ rostopic pub cmd_vel geometry_msgs/Twist -r 1 -- '[0.0,0.0,0.0]' '[0.0,0.0,0.0]'
```