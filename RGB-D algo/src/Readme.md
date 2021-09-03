This ros wrapper consists of several nodes for detecting social distancing breaches using an RFB-D camera.<br/>
It is being tested with ROS Noetic and Ubuntu 20.04<br/>
The steps needed to run it are:
1. Download and install ROS noetic
2. Create a ROS Workspace: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
3. In order to run the wrapper with a turtlebot robot, we will need to download the turtlebot packages:
  1. cd ~/catkin_ws/src/
  2. git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
  3. git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
  4. cd ~/catkin_ws && catkin_make
  5. git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
4. Clone mybot_gazebo into '<cd ~/catkin_ws/src/>' as well
5. '<cd ~/catkin_ws>' && catkin_make
6. export SVGA_VGPU10=0 (in case you are running on a VM)

Run the following commands: <br/>
Terminal 1: 2D and 3D object detection nodes <br/> 
  1. source devel/setup.bash
  2. roslaunch darknet_ros_3d darknet_ros_3d.launch
  
Terminal 2: Robot and Gazebo env <br/>
  1. source devel/setup.bash
  2. export TURTLEBOT3_MODEL=waffle
  3. roslaunch mybot_gazebo breach_detection.launch

Terminal 3: Tracking and computing breaches node  <br/>
  1. source devel/setup.bash
  2. roslaunch sort_track sort.launch
