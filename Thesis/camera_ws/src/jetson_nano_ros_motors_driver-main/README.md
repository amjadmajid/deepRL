 # ROS Node for Basic Jetbot Navigation
 
 1. install Adafruit library for TB6612/PCA9685 motor driver <br> `sudo apt install Adafruit-MotorHAT`
 2. gain access to the i2c bus <br> `sudo usermod -aG 12c $USER`
 3. reboot
 4. `cd ~/catkin_ws/src`
 5. `git clone https://github.com/robominded/jetson_nano_ros_motor_driver.git`
 6. `cd ~/catkin_ws`
 7. `catkin_make`
 8. to run jetson_nano_ros_motor_driver run <br> `roscore` in a terminal and  `rosrun jetson_basic_nav jetson_motors.py` in a different terminal.
 9. The package expects Twist messages. <br> For example you can install `sudo apt-get install ros-melodic-teleop-twist-keyboard
` to contorl the robot with your keyboard. 
