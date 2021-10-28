1. Instructions:

   DRL in Simulation:

   1. Open the robot model in rviz     and gazebo:

      - roslaunch jetbot_diff_drive      jetbot_rviz_gazebo.launch

   2. Create a map of our     simulation
      - roslaunch jetbot_diff_drive      jetbot_rviz_gazebo.launch
      - roslaunch hector_slam_launch      tutorial.launch
      - rosrun map_server map_saver      –f sim_env
      - Save the map in the maps      folder: robot/src/DRL_nav/maps

   4. Run the DRL training on the saved map (navigation with AMCL)
      - roslaunch jetbot_diff_drive      jetbot_rviz_gazebo_2.launch
      - roslaunch DRL_nav      tf_launch.launch
      - roslaunch      rf2o_laser_odometry rf2o_laser_odometry.launch
      - roslaunch DRL_nav      amcl_launch.launch
      - roslaunch DRL_nav      ddpg_stage_2sim.launch

    

   Run classical navigation on the real robot:

   1. On the robot:
      - roslaunch rplidar_ros      rplidar_s1.launch
      - rosrun jetbot_ros      jetson_motors.py

   3. On your PC:
      - roslaunch      classical_navigation tf_launch.launch
      - roslaunch      rf2o_laser_odometry rf2o_laser_odometry.launch
      - roslaunch      classical_navigation nav_launch.launch

    

   Run DDPG with AMCL on the robot:

   1. On the robot:
      - roslaunch rplidar_ros      rplidar_s1.launch
      - rosrun jetbot_ros      jetson_motors.py

   3. On the PC:
      - Create a map of the      environment
      - roslaunch DRL_nav      tf_launch_real.launch
      - roslaunch      rf2o_laser_odometry rf2o_laser_odometry.launch
      - roslaunch DRL_nav      amcl_launch.launch
      - roslaunch DRL_nav      ddpg_stage_2.launch
      - rostopic pub findObjects      custom_msgs/Object -r 1 -- -0.5 0.4 0.0
