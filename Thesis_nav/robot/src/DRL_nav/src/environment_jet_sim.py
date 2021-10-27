#!/usr/bin/env python3


import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

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
        self.odom_reset = rospy.Publisher('/syscommand', String, queue_size=2)
        # self.sub_odom = rospy.Subscriber('/rtabmap/odom', Odometry, self.getOdometry) #scanmatch_odom /rtabmap/odom
        self.sub_odom = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.odom_reset = rospy.Publisher('/syscommand', String, queue_size=2)
        self.pose_reset = rospy.ServiceProxy('/global_localization', Empty)
        self.odom_map = rospy.ServiceProxy('/rtabmap/reset_odom', Empty)
        self.pub_laser = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.stopped = 0
        self.action_dim = action_dim
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        #you can stop turtlebot by publishing an empty Twist
        #message
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance

    def getOdometry(self, odom):
        print(odom.pose.pose.position)
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
        min_range = 0.2 #6
        done = False

        scan_ranger = []
        int_ranges=[]


        # scan_ranger.append(scan.ranges[0])
        # scan_ranger.append(scan.ranges[36])
        # scan_ranger.append(scan.ranges[72])
        # scan_ranger.append(scan.ranges[108])
        # scan_ranger.append(scan.ranges[144])
        # scan_ranger.append(scan.ranges[180])
        # scan_ranger.append(scan.ranges[216])
        # scan_ranger.append(scan.ranges[252])
        # scan_ranger.append(scan.ranges[288])
        # scan_ranger.append(scan.ranges[324])

        scan_ranger.append(scan.ranges[0])
        scan_ranger.append(scan.ranges[18])
        scan_ranger.append(scan.ranges[36])
        scan_ranger.append(scan.ranges[54])
        scan_ranger.append(scan.ranges[72])

        scan_ranger.append(scan.ranges[90])
        scan_ranger.append(scan.ranges[108])
        scan_ranger.append(scan.ranges[126])
        scan_ranger.append(scan.ranges[144])
        scan_ranger.append(scan.ranges[162])

        scan_ranger.append(scan.ranges[180])
        scan_ranger.append(scan.ranges[198])
        scan_ranger.append(scan.ranges[216])
        scan_ranger.append(scan.ranges[234])
        scan_ranger.append(scan.ranges[252])

        scan_ranger.append(scan.ranges[270])
        scan_ranger.append(scan.ranges[288])
        scan_ranger.append(scan.ranges[306])
        scan_ranger.append(scan.ranges[324])
        scan_ranger.append(scan.ranges[342])

        int_ranges.append(scan.intensities[0])
        int_ranges.append(scan.intensities[18])
        int_ranges.append(scan.intensities[36])
        int_ranges.append(scan.intensities[54])
        int_ranges.append(scan.intensities[72])

        int_ranges.append(scan.intensities[90])
        int_ranges.append(scan.intensities[108])
        int_ranges.append(scan.intensities[126])
        int_ranges.append(scan.intensities[144])
        int_ranges.append(scan.intensities[162])

        int_ranges.append(scan.intensities[180])
        int_ranges.append(scan.intensities[198])
        int_ranges.append(scan.intensities[216])
        int_ranges.append(scan.intensities[234])
        int_ranges.append(scan.intensities[252])

        int_ranges.append(scan.intensities[270])
        int_ranges.append(scan.intensities[288])
        int_ranges.append(scan.intensities[306])
        int_ranges.append(scan.intensities[324])
        int_ranges.append(scan.intensities[342])


        # int_ranges.append(scan.intensities[0])
        # int_ranges.append(scan.intensities[36])
        # int_ranges.append(scan.intensities[72])
        # int_ranges.append(scan.intensities[108])
        # int_ranges.append(scan.intensities[144])
        # int_ranges.append(scan.intensities[180])
        # int_ranges.append(scan.intensities[216])
        # int_ranges.append(scan.intensities[252])
        # int_ranges.append(scan.intensities[288])
        # int_ranges.append(scan.intensities[324])

        scann = LaserScan()
        current_time = rospy.Time.now()
        scann.header.stamp = current_time
        scann.header.frame_id = 'rplidar_base_scan'
        scann.angle_min = 0.0
        scann.angle_max = 6.28319
        scann.angle_increment = 0.3141595
        scann.range_min = 0.11999999731779099
        scann.range_max = 3.5
        scann.ranges = scan_ranger 
        scann.intensities = int_ranges
        
        self.pub_laser.publish(scann)

        for i in range(len(scann.ranges)):
            if scann.ranges[i] == float('Inf') or scann.ranges[i] == float('inf'):
                scan_range.append(3.5)
            elif np.isnan(scann.ranges[i]) or scann.ranges[i] == float('nan'):
                scan_range.append(0)
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
        current_distance = state[-1]
        heading = state[-2]

        distance_rate = (self.past_distance - current_distance) 
        if distance_rate > 0:
            ob_reward = 0.

        if distance_rate <= 0:
            ob_reward = -5
            #reward = 0.

        angle_reward = math.pi - abs(heading)
        
        reward = 500.*distance_rate + 3.*angle_reward + ob_reward
        self.past_distance = current_distance

        # a, b, c, d = float('{0:.3f}'.format(self.position.x)), float('{0:.3f}'.format(self.past_position.x)), float('{0:.3f}'.format(self.position.y)), float('{0:.3f}'.format(self.past_position.y))
        # if a == b and c == d:
 
        #     self.stopped += 1
        #     if self.stopped == 20:
        #         rospy.loginfo('Robot is in the same 20 times in a row')
        #         self.stopped = 0
        #         done = True
        # else:
        #     # rospy.loginfo('\n>>>>> not stopped>>>>>\n')
        #     self.stopped = 0

        if done:
            rospy.loginfo("Collision!!")
            reward = -500.
            #reward = -10.
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            # reward = 500.
            reward = 1000.
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
        self.pub_cmd_vel.publish(vel_cmd)

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
        
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_world()
            #self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        self.pose_reset()
        #self.odom_reset.publish("reset")

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

        self.goal_distance = self.getGoalDistace()
        state, _ = self.getState(data, [0]*self.action_dim)

        return np.asarray(state)