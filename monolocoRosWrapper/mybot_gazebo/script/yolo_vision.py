#!/usr/bin/env python

# This node listens to bounding boxes from the darknet yolo object detector and uses the camera depth point 
# cloud to calculate the 3D positions relative to the robot

# Subscribes to point cloud from the camera depth sensor:
#     /camera/depth_registered/points - Depth data from camera

# Subscribes to bounding boxes:
#     /darknet_ros/bounding_boxes - YOLO object detection

# Publishes to:
#     /find_objects - object_detection.msg.Object array of 3D positions of objects for
#                     the given bounding boxes
#     /find_objects_debug - PointStamped x, y, z of last object found

# import cv2
import math
import rospy
import datetime
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg  import Twist, Vector3
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from darknet_ros_msgs.msg import Object
import message_filters
from darknet_ros_msgs.msg import DetectedObjectMsg
from darknet_ros_msgs.msg import DetectedObjectsMsg
from geometry_msgs.msg import Point
rospy.loginfo('People detection starting')

class FindObjectsNode:
    def __init__(self):
        rospy.init_node('findObjects')
        self.init_subscribers()
        self.init_publishers()
        self._check_camera_depth_points_ready()
        self.maxpoints = rospy.get_param("~maxpoints", 50)
        self.debug = rospy.get_param("~debug", False)
        self.bridge = CvBridge()
        self.detected_objects = []

    def _check_camera_depth_points_ready(self):
        self.lastCloud = None
        rospy.logdebug("Waiting for /camera/depth/points to be READY...")
        while self.lastCloud is None and not rospy.is_shutdown():
            try:
                self.lastCloud = rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=10.0)
                rospy.logdebug("Current /camera/depth/points READY=>")

            except:
                rospy.logerr("Current /camera/depth/points not ready yet, retrying for getting camera_depth_points")
        return self.lastCloud

    def init_subscribers(self):
        
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image, buff_size=2**28)
        self.box_sub = message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, buff_size=2**28)
        self.pointcloud_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2, buff_size=2**28)

        # try to synchronize the 3 topics as closely as possible for higher XYZ accuracy
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.box_sub, \
            self.pointcloud_sub], queue_size=20, slop=0.03)
        self.time_sync.registerCallback(self.callback)

        rospy.loginfo("Subscribers Synchronized Successfully")

        #rospy.Subscriber("/camera/depth/points", PointCloud2,self.callback_point)
        #rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes,self.callback_box)

    def init_publishers(self):
        self.pub = rospy.Publisher('find_objects', Object, queue_size=1)
        #self.detected_objects_3d_pos_pub = rospy.Publisher('/object_detector/detected', DetectedObjectsMsg, queue_size=1)

        # if (self.ros_config['enable_debug_image']):
        #     self.debug_img_pub = rospy.Publisher(self.publishers['debug_image']['topic'], \
        #         Image, queue_size=self.publishers['debug_image']['queue_size'])


    def callback(self, ros_img,ros_box, ros_pointcloud):
        self.image_cb(ros_img,ros_box)
        self.pointcloud_cb(ros_pointcloud)   

    def image_cb(self, ros_img, ros_box):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            self.cv_image_timestamp = ros_img.header.stamp
            self.process(ros_box)
        except CvBridgeError as e:
            print(e)

    def pointcloud_cb(self, ros_pointcloud):
        self.pointcloud = ros_pointcloud
        self.pointcloud_timestamp= ros_pointcloud.header.stamp

    # perform object detection and calculate xyz coordinate of object from pointcloud
    def process(self,data):
        # self.detected_objects = []

        max_time_difference = rospy.Duration(0.2)
        if (hasattr(self,'pointcloud')): 
            for b in data.bounding_boxes:
                rospy.loginfo('Here!!!!!!!!!!!!!!!')
                points = []
                dist2 = []
                point= Point()
                #xyz_coord = list(pc2.read_points(self.pointcloud, skip_nans=True, field_names=("x", "y", "z")))
                for p in pc2.read_points(self.pointcloud, skip_nans=True,field_names=("x", "y", "z")):
                        points.append((p[0],p[1],p[2]))
                        dist2.append(p[0]**2 + p[1]**2 + p[2]**2)
                        
                minindex = np.argmin(np.array(dist2))
                loc = points[minindex]
                point.x=loc[0]
                point.y=loc[1]
                point.z=loc[2]
                msg = Object()
                msg.label = b.Class
                msg.point = point
                # msg.x = loc[0]
                # msg.y = loc[1]
                # msg.z = loc[2]
                self.pub.publish(msg)
                # msg.x = points[0]
                # msg.y = points[1]
                # msg.z = points[2]
                # self.pub.publish(msg)


        # if (hasattr(self,'pointcloud') and  abs(self.cv_image_timestamp-self.pointcloud_timestamp) < max_time_difference):
        #     # Note: ML detection can take significant time
        #     self.detected_objects = self.object_detector.detect(self.cv_image) 
        #     for b in data.bounding_boxes:
        #         rospy.loginfo('Here!!!!!!!!!!!!!!!')
        #         for p in pc2.read_points(self.pointcloud, skip_nans=True,field_names=("x", "y", "z"), uvs=[b.center]):
        #             points.append((p[0],p[1],p[2]))
        #         msg = Object()
        #         msg.x = points[0]
        #         msg.y = points[1]
        #         msg.z = points[2]
        #         self.pub.publish(msg)
        # else:
        #     rospy.logwarn("Synchronization Issue\nImage and PointCloud not Synchronized")

    # def callback_point(self, cloud):
    #     self.lastCloud = cloud

    # def callback_box(self, data):
    #     if self.lastCloud:
    #         for b in data.bounding_boxes:
    #             uvs = []
    #             xmin=b.xmin
    #             xmax=b.xmax
    #             ymin=b.ymin
    #             ymax=b.ymax
    #             w=xmax-xmin
    #             h=ymax-ymin
    #             if w*h > self.maxpoints and w > 0 and h > 0:
    #                 wnew = math.ceil(math.sqrt(1.0*self.maxpoints*w/h))
    #                 hnew = math.ceil(1.0*wnew*h/w)
    #                 xcenter = math.floor(xmin + 0.5*w)
    #                 ycenter = math.floor(ymin + 0.5*h)
    #                 xmin = int(math.floor(xcenter - 0.5*wnew))  #left
    #                 xmax = int(math.floor(xcenter + 0.5*wnew))  #right
    #                 ymin = int(math.floor(ycenter - 0.5*hnew))  #top
    #                 ymax = int(math.floor(ycenter + 0.5*hnew))  #bot

    #             for x in range(xmin, xmax+1):
    #                 for y in range(ymin, ymax+1):
    #                     uvs.append((x,y))

    #             points = []
    #             dist2 = []

    #             for p in pc2.read_points(self.lastCloud, field_names=("x","y","z"),uvs=uvs, skip_nans=False):
    #                 points.append((p[0],p[1],p[2]))
    #                 dist2.append(p[0]**2 + p[1]**2 + p[2]**2)

    #             if len(points) > 0:
    #                 minindex = np.argmin(np.array(dist2))
    #                 loc = points[minindex]
    #                 msg = Object()
    #                 msg.label = b.Class
    #                 msg.x = loc[0]
    #                 msg.y = loc[1]
    #                 msg.z = loc[2]
    #                 self.pub.publish(msg)
    #             else:
    #                 rospy.loginfo("all points are NaN in bounding box")

    #             # # Read points from a L{sensor_msgs.PointCloud2} message.
    #             # # @param cloud: The point cloud to read from.
    #             # # @param field_names: The names of fields to read. If None, read all fields. [default: None]
    #             # # @param skip_nans: If True, then don't return any point with a NaN value.
    #             # # @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    #             # # http://docs.ros.org/en/jade/api/sensor_msgs/html/point__cloud2_8py_source.html
                
    #             #     if self.debug:
    #             #         msg = PointStamped()
    #             #         # Apparently supposed to be the reference frame for when plotted in rviz
    #             #         msg.header.frame_id = self.target
    #             #         msg.point.x = p[0]
    #             #         msg.point.y = p[1]
    #             #         msg.point.z = p[2]
    #             #         self.debugPoint.publish(msg)

    #             #     rospy.logdebug("%s x %f y %f z %f" %(b.Class,p[0],p[1],p[2]))
    #             # else:
    #             #     rospy.loginfo("all points are NaN in bounding box")


if __name__ == '__main__':
    try:
        node = FindObjectsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
