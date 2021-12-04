#!/usr/bin/env python3


import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
import copy


class Env():
    def __init__(self, action_dim,x,y):
        self.goal_x = x
        self.goal_y = y
        self.counter=0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.odom_reset = rospy.Publisher('/syscommand', String, queue_size=2)
        self.sub_odom = rospy.Subscriber('/scanmatch_odom', Odometry, self.getOdometry) #scanmatch_odom /rtabmap/odom
        self.odom_map = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
        self.pub_laser = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        self.past_vel=0.
        self.vel=0
        self.neg=0
        self.pos=0
        self.change=0
        self.orien=0
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance
        return goal_distance

    def getOdometry(self, odom):
        self.past_position = copy.deepcopy(self.position)
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        self.orien=yaw
        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        min_range = 0.16
        done = False

        scan_ranger = []
        int_ranges=[]


        scan_ranger.append(scan.ranges[18])
        scan_ranger.append(scan.ranges[36])
        scan_ranger.append(scan.ranges[54])
        scan_ranger.append(scan.ranges[72])
        scan_ranger.append(scan.ranges[90])
        scan_ranger.append(scan.ranges[-18])
        scan_ranger.append(scan.ranges[-36])
        scan_ranger.append(scan.ranges[-54])
        scan_ranger.append(scan.ranges[-72])
        scan_ranger.append(scan.ranges[-90])

        int_ranges.append(scan.intensities[18])
        int_ranges.append(scan.intensities[36])
        int_ranges.append(scan.intensities[54])
        int_ranges.append(scan.intensities[72])
        int_ranges.append(scan.intensities[90])
        int_ranges.append(scan.intensities[-18])
        int_ranges.append(scan.intensities[-36])
        int_ranges.append(scan.intensities[-54])
        int_ranges.append(scan.intensities[-72])
        int_ranges.append(scan.intensities[-90])

        scann = LaserScan()
        current_time = rospy.Time.now()
        scann.header.stamp = current_time
        scann.header.frame_id = 'laser'
        scann.angle_min = -1.5708
        scann.angle_max = 1.5708
        scann.angle_increment = 0.314159
        scann.time_increment = 0.00012963535846211016
        scann.range_min = 0.15000000596046448
        scann.range_max = 40.0
        scann.ranges = scan_ranger 
        scann.intensities = int_ranges
        self.pub_laser.publish(scann)


        for i in range(len(scann.ranges)):
            if scann.ranges[i] == float('Inf') or scann.ranges[i] == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(scann.ranges[i]) or scann.ranges[i] == float('nan'):
                scan_range.append(0)
            elif scann.ranges[i] > 3.5:
                scan_range.append(3.5)
            else:
                scan_range.append(scann.ranges[i])

        if min_range > min(scan_range) > 0:
            done = True

        for pa in past_action:
            scan_range.append(pa)

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        # current_distance = self.getGoalDistace()
        if current_distance < 0.15:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done):
        target = 180
        kp=0.5
        current_distance = state[-1]
        heading = state[-2]
        ob_reward=0
        swing_penalty=0
        bad_penalty=0
        #print('cur:', current_distance, self.past_distance)


        distance_rate = (self.past_distance - current_distance) 
        if distance_rate > 0:
            reward = 200.*distance_rate
        #if distance_rate == 0:
        #    reward = -10.
        if distance_rate <= 0:
            reward = -8.
        #angle_reward = math.pi - abs(heading)
        #print('d', 500*distance_rate)
        #reward = 500.*distance_rate #+ 3.*angle_reward
        self.past_distance = current_distance
        # if (self.vel<0 and self.past_vel<=0):
        #     self.neg=self.neg+1
        # elif (self.vel>0 and self.neg!=0):
        #     if (self.neg==2 or self.neg==3 or self.neg==4 or self.neg==5):
        #         self.neg=0
        #         self.pos=self.pos+1
        #         bad_penalty=-5
        # elif (self.vel<0 and self.pos!=0):
        #     if (self.pos==2 or self.pos==3 or self.pos==4 or self.pos==5):
        #         self.pos=0
        #         self.neg=self.neg+1
        #         bad_penalty=-5
        # elif (self.vel>0 and self.past_vel>=0):
        #     self.pos=self.pos+1

        # check=self.vel*self.past_vel
        # if (check<0):
        #      
        #     # print(swing_penaltyswing)

        # self.counter+=1
        # check=self.vel*self.past_vel
        # if (check<0):
        #     self.change+=1
        #     bad_penalty=-5*(abs(self.vel-self.past_vel))

        # if (self.counter==15):
        #     if (self.change>2):
        #         swing_penalty=-3*self.change
        #     # elif self.change==0:
        #     #     swing_penalty=-5
        #     self.counter=0
        #     self.change=0
        
        reward=swing_penalty+ob_reward+bad_penalty    
        # self.past_vel=self.vel
        if done:
            rospy.loginfo("Collision!!")
            reward = -300.
            done=True
            self.pub_cmd_vel.publish(Twist())
        command =Twist()
        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 500.
            self.pub_cmd_vel.publish(Twist())
            done=True
            # if (self.counter==0):
            #     self.goal_x = 0.0
            #     self.goal_y =0.0
            #     self.counter+=1
            #     self.get_goalbox = False
            # elif self.counter==1:
            #     # self.goal_x = 0.01
            #     # self.goal_y =0.0
            #     self.counter+=1
            #     self.get_goalbox = False          
            #     target_rad = target*math.pi/180      
            #     command.angular.z = kp * (target_rad-self.orien)
            #     self.pub_cmd_vel.publish(command)
            #     print("rotating")
            # else:
            #     self.counter=0
            #     done=True

        return reward, done

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel

        self.pub_cmd_vel.publish(vel_cmd)
        self.vel=ang_vel
        # print(ang_vel)
        
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data, past_action)
        reward, done = self.setReward(state, done)
        return np.asarray(state), reward, done

    def reset(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
            
        self.goal_distance = self.getGoalDistace()
        state, _ = self.getState(data, [0]*self.action_dim)

        return np.asarray(state)