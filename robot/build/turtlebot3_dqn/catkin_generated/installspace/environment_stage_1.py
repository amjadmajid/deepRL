#!/usr/bin/env python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
import random
import time
import os
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('turtlebot3_dqn/src/turtlebot3_dqn',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        if self.stage != 4:
            while position_check:
                goal_x = random.randrange(-12, 13) / 10.0
                goal_y = random.randrange(-12, 13) / 10.0
                if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        else:
            while position_check:
                goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
                goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

                self.index = random.randrange(0, 13)
                print(self.index, self.last_index)
                if self.last_index == self.index:
                    position_check = True
                else:
                    self.last_index = self.index
                    position_check = False

                self.goal_position.position.x = goal_x_list[self.index]
                self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y


class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('jetbot_diff_drive_controller/cmd_vel', Twist, queue_size=5)
        self.odom_reset = rospy.Publisher('/syscommand', String, queue_size=2)
        self.sub_odom = rospy.Subscriber('/rtabmap/odom', Odometry, self.getOdometry)
        # self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.remapLaser)
        self.pub_laser = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.odom_map = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
        self.respawn_goal = Respawn()

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def remapLaser(self,scan):
        scan_ranger = []
        scan_ranger.append(scan.ranges[0])
        scan_ranger.append(scan.ranges[15])
        scan_ranger.append(scan.ranges[30])
        scan_ranger.append(scan.ranges[45])
        scan_ranger.append(scan.ranges[60])
        scan_ranger.append(scan.ranges[75])
        scan_ranger.append(scan.ranges[90])
        scan_ranger.append(scan.ranges[105])
        scan_ranger.append(scan.ranges[120])
        scan_ranger.append(scan.ranges[135])
        scan_ranger.append(scan.ranges[150])
        scan_ranger.append(scan.ranges[165])
        scan_ranger.append(scan.ranges[180])
        scan_ranger.append(scan.ranges[195])
        scan_ranger.append(scan.ranges[210])
        scan_ranger.append(scan.ranges[225])
        scan_ranger.append(scan.ranges[240])
        scan_ranger.append(scan.ranges[255])
        scan_ranger.append(scan.ranges[270])
        scan_ranger.append(scan.ranges[285])
        scan_ranger.append(scan.ranges[300])
        scan_ranger.append(scan.ranges[315])
        scan_ranger.append(scan.ranges[330])
        scan_ranger.append(scan.ranges[345])
        scann = LaserScan()
        current_time = rospy.Time.now()
        scann.header.stamp = current_time
        scann.header.frame_id = 'rplidar_base_scan'
        scann.angle_min = 0.0
        scann.angle_max = 6.28319
        scann.angle_increment = 0.2731821835041046
        scann.time_increment = 0.0
        scann.range_min = 0.11999999731779099
        scann.range_max = 3.5
        scann.ranges = scan_ranger #scan.ranges[0:359]#s
        scann.intensities = scan.intensities[0:359]
        self.pub_laser.publish(scann)

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
        scan_ranger = []
        int_ranges=[]


        scan_ranger.append(scan.ranges[0])
        scan_ranger.append(scan.ranges[15])
        scan_ranger.append(scan.ranges[30])
        scan_ranger.append(scan.ranges[45])
        scan_ranger.append(scan.ranges[60])

        scan_ranger.append(scan.ranges[75])
        scan_ranger.append(scan.ranges[90])
        scan_ranger.append(scan.ranges[105])
        scan_ranger.append(scan.ranges[120])
        scan_ranger.append(scan.ranges[135])

        scan_ranger.append(scan.ranges[150])
        scan_ranger.append(scan.ranges[165])
        scan_ranger.append(scan.ranges[180])
        scan_ranger.append(scan.ranges[195])
        scan_ranger.append(scan.ranges[210])

        scan_ranger.append(scan.ranges[225])
        scan_ranger.append(scan.ranges[240])
        scan_ranger.append(scan.ranges[255])
        scan_ranger.append(scan.ranges[270])
        scan_ranger.append(scan.ranges[285])

        scan_ranger.append(scan.ranges[300])
        scan_ranger.append(scan.ranges[315])
        scan_ranger.append(scan.ranges[330])
        scan_ranger.append(scan.ranges[345])

        int_ranges.append(scan.intensities[0])
        int_ranges.append(scan.intensities[15])
        int_ranges.append(scan.intensities[30])
        int_ranges.append(scan.intensities[45])
        int_ranges.append(scan.intensities[60])

        int_ranges.append(scan.intensities[75])
        int_ranges.append(scan.intensities[90])
        int_ranges.append(scan.intensities[105])
        int_ranges.append(scan.intensities[120])
        int_ranges.append(scan.intensities[135])

        int_ranges.append(scan.intensities[150])
        int_ranges.append(scan.intensities[165])
        int_ranges.append(scan.intensities[180])
        int_ranges.append(scan.intensities[195])
        int_ranges.append(scan.intensities[210])

        int_ranges.append(scan.intensities[225])
        int_ranges.append(scan.intensities[240])
        int_ranges.append(scan.intensities[255])
        int_ranges.append(scan.intensities[270])
        int_ranges.append(scan.intensities[285])

        int_ranges.append(scan.intensities[300])
        int_ranges.append(scan.intensities[315])
        int_ranges.append(scan.intensities[330])
        int_ranges.append(scan.intensities[345])

        scann = LaserScan()
        current_time = rospy.Time.now()
        scann.header.stamp = current_time
        scann.header.frame_id = 'rplidar_base_scan'
        scann.angle_min = 0.0
        scann.angle_max = 6.28319
        scann.angle_increment = 0.2617995833
        scann.range_min = 0.11999999731779099
        scann.range_max = 3.5
        scann.ranges = scan_ranger 
        scann.intensities = int_ranges
        
        self.pub_laser.publish(scann)

        for i in range(len(scann.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scann.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scann.ranges[i])

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

        distance_rate = None
        while distance_rate is None:
            try:
                distance_rate = 2 ** (current_distance / self.goal_distance)
            except:
                distance_rate=2
        
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        if done:
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 200
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def step(self, action):
        max_angular_vel = 0.55 #1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.09 #0.09 #0.15
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
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            #self.reset_proxy()
            self.reset_world()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        
        # self.odom_reset.publish("reset")
        self.odom_map()

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)