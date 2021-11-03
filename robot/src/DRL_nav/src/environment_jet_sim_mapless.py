#!/usr/bin/env python3

import rospy
import math
import numpy as np
from math import pi
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
world = False
if world:
    from respawnGoal_custom_worlds import Respawn
else:
    from respawnGoal import Respawn
import copy
target_not_movable = False

class Env():
    def __init__(self, action_dim=2):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('jetbot_diff_drive_controller/cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('/rtabmap/odom', Odometry, self.getOdometry)
        self.odom_reset = rospy.Publisher('/syscommand', String, queue_size=2)  
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.odom_map = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        self.past_vel=0.
        self.vel=0
        self.neg=0
        self.pos=0
        self.counter=0
        self.change=0
        
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

        heading = goal_angle - yaw

        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        min_range = 0.20 
        done = False

        #Get 10 Lidar measurements out of 360 for the current state
        scan_range.append(max(scan[-90], scan[-91], scan[-92]))
        scan_range.append(max(scan[-70], scan[-71], scan[-72]))
        scan_range.append(max(scan[-50], scan[-51], scan[-52]))
        scan_range.append(max(scan[-30], scan[-31], scan[-32]))
        scan_range.append(max(scan[-10], scan[-11], scan[-12]))
        scan_range.append(max(scan[8], scan[9], scan[10]))
        scan_range.append(max(scan[28], scan[29], scan[30]))
        scan_range.append(max(scan[48], scan[49], scan[50]))
        scan_range.append(max(scan[68], scan[69], scan[70]))
        scan_range.append(max(scan[88], scan[89], scan[90]))
        
        for i in range(len(scan_range)):
            if scan_range[i] == 0.0:
                scan_range[i] = 3.5
            elif scan_range[i] > 3.5:
                scan_range[i] = 3.5
            scan_range[i] = round(scan_range[i], 3)

        for j in range(len(scan_range)):
            if scan_range[j] == float('Inf'):
                scan_range[j] = 3.5
            elif np.isnan(scan_range[j]):
                scan_range = 0.

        if min_range > min(scan_range) > 0:
            done = True

        for pa in past_action:
            scan_range.append(pa)

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)

        if current_distance < 0.15:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done):
        current_distance = state[-1]
        heading = state[-2]
        ob_reward=0
        swing_penalty=0
        bad_penalty=0

        distance_rate = (self.past_distance - current_distance) 
        if distance_rate > 0:
            ob_reward = 200.*distance_rate

        if distance_rate <= 0:
            ob_reward = -8.

        self.past_distance = current_distance

        a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float('{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        if a == b and c == d:
            self.stopped += 1
            if self.stopped == 20:
                rospy.loginfo('Robot is in the same 20 times in a row')
                self.stopped = 0
                done = True
        else:
            self.stopped = 0

        # Uncomment the below to add penalties for swinging

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
        self.past_vel=self.vel

        if done:
            rospy.loginfo("Collision!!")
            reward = -300.
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 500.
            self.pub_cmd_vel.publish(Twist())
            if world:
                self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True, running=True)
                if target_not_movable:
                    self.reset()
            else:
                self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward, done

    def step(self, action, past_action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.vel=ang_vel
        print(ang_vel)
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data.ranges, past_action)
        reward, done = self.setReward(state, done)

        return np.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_world()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        # self.odom_reset.publish("reset") # Reset Hector odometry
        self.odom_map() # Reset RTBMAP odometry

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

        self.goal_distance = self.getGoalDistace()
        state, _ = self.getState(data.ranges, [0]*self.action_dim)

        return np.asarray(state)
