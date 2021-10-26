On the Jetbot:
1. sudo chmod 666 /dev/ttyUSB0
2. roslaunch rplidar_ros rplidar_s1.launch
3. rosrun jetbot_ros jetson_motors.py

On the PC - "robot" folder:

​	#It is still in the hector launch file however we are not running hector slam

1. roslaunch hector_slam_launch tutorial.launch

2. roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch

3. rosrun map_server map_server maps/test.yaml

   #Run AMCL and move_base

4. roslaunch project ddpg_stage_2.launch
