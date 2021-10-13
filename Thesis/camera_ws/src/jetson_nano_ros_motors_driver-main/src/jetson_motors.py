#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from geometry_msgs.msg import Twist

def set_speed(motor_id, value):
	max_pwm = 70.0 #115.0
	speed = int(min(abs(value * max_pwm) , max_pwm))

	if motor_id == 1:
		motor = left_motor 
	elif motor_id == 2:
		motor = right_motor 
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)

def stop_motors():
	left_motor.setSpeed(0)
	right_motor.setSpeed(0)

	left_motor.run(Adafruit_MotorHAT.RELEASE)
	right_motor.run(Adafruit_MotorHAT.RELEASE)

left_motor_speed = 0 
right_motor_speed = 0
def on_cmd_vel(msg):

    global left_motor_speed
    global right_motor_speed

    rospy.loginfo(rospy.get_caller_id() + 'linear.x=%f', msg.linear.x)
    rospy.loginfo(rospy.get_caller_id() + 'angular.z=%f', msg.angular.x)
    
    if msg.angular.z ==0: 
        left_motor_speed = msg.linear.x
        right_motor_speed = msg.linear.x
    else:
        if msg.angular.z > 0: # the assumption is robot goes right
            left_motor_speed = 0.7
            right_motor_speed = 0.3
        else:
            left_motor_speed = 0.3
            right_motor_speed = 0.7

    set_speed(left_motor_id, left_motor_speed)
    set_speed(right_motor_id, right_motor_speed)

if __name__ == '__main__':

        # get access to the motor driver
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	left_motor_id = 1
	right_motor_id = 2

	left_motor = motor_driver.getMotor(left_motor_id)
	right_motor = motor_driver.getMotor(right_motor_id)

	# initialize ros node
	rospy.init_node('jetbot_motors')
        rospy.Subscriber('cmd_vel', Twist, on_cmd_vel)	
        
        #spin while waiting for driving commands
	rospy.spin()

        stop_motors()

