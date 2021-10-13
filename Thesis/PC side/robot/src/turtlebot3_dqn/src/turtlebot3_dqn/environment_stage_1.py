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

class Env():
    def __init__(self, action_size,x,y):
        self.goal_x = x
        self.goal_y =y
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/scanmatch_odom', Odometry, self.getOdometry)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        scan_range.append(max(scan.ranges[-178], scan.ranges[-179], scan.ranges[-180]))
        scan_range.append(max(scan.ranges[-163], scan.ranges[-164], scan.ranges[-165]))
        scan_range.append(max(scan.ranges[-148], scan.ranges[-149], scan.ranges[-150]))
        scan_range.append(max(scan.ranges[-133], scan.ranges[-134], scan.ranges[-135]))
        scan_range.append(max(scan.ranges[-118], scan.ranges[-119], scan.ranges[-120]))
        scan_range.append(max(scan.ranges[-103], scan.ranges[-104], scan.ranges[-105]))
        scan_range.append(max(scan.ranges[-88], scan.ranges[-89], scan.ranges[-90]))
        scan_range.append(max(scan.ranges[-73], scan.ranges[-74], scan.ranges[-75]))
        scan_range.append(max(scan.ranges[-58], scan.ranges[-59], scan.ranges[-60]))
        scan_range.append(max(scan.ranges[-43], scan.ranges[-44], scan.ranges[-45]))
        scan_range.append(max(scan.ranges[-28], scan.ranges[-29], scan.ranges[-30]))
        scan_range.append(max(scan.ranges[-13], scan.ranges[-14], scan.ranges[-15]))
        scan_range.append(max(scan.ranges[13], scan.ranges[14], scan.ranges[15]))
        scan_range.append(max(scan.ranges[28], scan.ranges[29], scan.ranges[30]))
        scan_range.append(max(scan.ranges[43], scan.ranges[44], scan.ranges[45]))
        scan_range.append(max(scan.ranges[58], scan.ranges[59], scan.ranges[60]))
        scan_range.append(max(scan.ranges[73], scan.ranges[74], scan.ranges[75]))
        scan_range.append(max(scan.ranges[88], scan.ranges[89], scan.ranges[90]))
        scan_range.append(max(scan.ranges[103], scan.ranges[104], scan.ranges[105]))
        scan_range.append(max(scan.ranges[118], scan.ranges[119], scan.ranges[120]))
        scan_range.append(max(scan.ranges[133], scan.ranges[134], scan.ranges[135]))
        scan_range.append(max(scan.ranges[148], scan.ranges[149], scan.ranges[150]))
        scan_range.append(max(scan.ranges[163], scan.ranges[164], scan.ranges[165]))
        scan_range.append(max(scan.ranges[178], scan.ranges[179], scan.ranges[180]))

        for i in range(len(scan_range)):
            if scan_range[i] == 0.0:
                scan_range[i] = 3.5
            elif scan_range[i] > 3.5:
                scan_range[i] = 3.5
            scan_range[i] = round(scan_range[i], 3)
        #print('vai:', scan_range)

        for j in range(len(scan_range)):
            if scan_range[j] == float('Inf'):
                scan_range[j] = 3.5
            elif np.isnan(scan_range[j]):
                scan_range = 0.

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            # self.goal_x=2
            # self.goal_y=2
            # self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    def reset(self):
        # rospy.wait_for_service('gazebo/reset_simulation')
        # try:
        #     self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #     print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        # if self.initGoal:
        #     self.goal_x, self.goal_y = self.respawn_goal.getPosition()
        #     self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)