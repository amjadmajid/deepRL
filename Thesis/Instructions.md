Install Intel® RealSense™ SDK 2.0 from Source (Jetson devices)

1. Clone this repository: https://github.com/jetsonhacks/installRealSenseSDK
2. cd installRealSenseSDK
3. $ ./buildLibrealsense.sh
4. Test out the installation: "realsense-viewer"
5. Clone this repository: https://github.com/JetsonHacksNano/installRealSenseROS
6. cd installRealSenseROS
7. $ ./installRealSenseROS.sh <catkin_ws_name>
8. Download https://github.com/ros-drivers/rgbd_launch/tree/noetic-devel into the catkin source
9. Run catkin_make
10. source devel/setup.bash
11. roslaunch realsense2_camera rs_rgbd.launch initial_reset:=true

######### Instructions only for running DRL navigation: ##########

On the Jetbot- "camera_ws" folder:
1. sudo chmod 666 /dev/ttyUSB0
2. roslaunch rplidar_ros rplidar_s1.launch
3. rosrun jetbot_ros jetson_motors.py

On the PC - "robot" folder:
1. roslaunch hector_slam_launch tutorial.launch
2. rostopic pub findObjects darknet_ros_msgs/Object -r 1 -- -1.5 0.15 0.0 (Navigate towards x=-1.5 and y=0.15)
3. roslaunch project ddpg_stage_2.launch
4. rostopic pub cmd_vel geometry_msgs/Twist -r 1 -- '[0.0,0.0,0.0]' '[0.0,0.0,0.0]' (Use it to stop the robot)

######### Instructions for navigation and detection: ##########

On the Jetbot- "camera_ws" folder:
1. sudo chmod 666 /dev/ttyUSB0
2. roslaunch rplidar_ros rplidar_s1.launch
3. rosrun jetbot_ros jetson_motors.py
4. roslaunch darknet_ros_3d darknet_ros_3d.launch
5. roslaunch realsense2_camera rs_rgbd.launch initial_reset:=true
6. roslaunch sort_track sort.launch

On the PC - "robot" folder:

1. roslaunch hector_slam_launch tutorial.launch
2. roslaunch project ddpg_stage_2.launch
3. rostopic pub cmd_vel geometry_msgs/Twist -r 1 -- '[0.0,0.0,0.0]' '[0.0,0.0,0.0]' (Use it to stop the robot)
