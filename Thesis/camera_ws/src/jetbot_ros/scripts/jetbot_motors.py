#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String




# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 150.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)


# directional commands (degree, speed)
def on_cmd_dir(msg):
	# rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)
	rospy.loginfo("Received a /cmd_vel message!")
	rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
	rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	x=msg.linear.x
	y=msg.angular.z/10
	if x>0 and y<0: #backward right
		rospy.loginfo(rospy.get_caller_id() + ', backward right (left, right)=(%s,%s)' % ((abs(y)+0.1), (0.2+y+0.1)))
		set_speed(motor_left_ID, (abs(y)+0.1))
		set_speed(motor_right_ID, (0.2+y+0.1))
	elif x>0 and y>0: #backward left
		rospy.loginfo(rospy.get_caller_id() + ', backward left (left, right)=(%s,%s)' % ((0.2-y+0.1), (y+0.1)))
		set_speed(motor_left_ID, (0.2-y+0.1))
		set_speed(motor_right_ID, (y+0.1))
	elif x<0 and y>0: #forward left
		rospy.loginfo(rospy.get_caller_id() + ', forward left (left, right)=(%s,%s)' % ((-(0.2-y)-0.1), -(y+0.1)))
		set_speed(motor_left_ID, (-(0.2-y)-0.1))
		set_speed(motor_right_ID, -(y+0.1))
	elif x<0 and y<0: #forward right
		rospy.loginfo(rospy.get_caller_id() + ', forward right (left, right)=(%s,%s)' % (y-0.1, (-(0.2+y)-0.1)))
		set_speed(motor_left_ID, y-0.1)
		set_speed(motor_right_ID, (-(0.2+y)-0.1))
	else:
		all_stop()

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('cmd_vel', Twist, on_cmd_dir)
	rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()
