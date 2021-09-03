#!/usr/bin/env python3

"""
ROS node to track objects using SORT and YOLOv3 (darknet_ros)
"""


import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ComputeBoxes,ComputeBox
from sort import sort 
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sort_track.msg import IntList
from darknet_ros_msgs.msg import Object
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64MultiArray
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters
import math 
import pandas as pd
df_breaches = pd.DataFrame(columns=('Person_Breaching', 'nb_breaches', 'XCoord','YCoord')) #Person_Breaching is a list of persons
df = pd.DataFrame(columns=('ID','Person','Xmax', 'Xmin', 'Ymax', 'Ymin'))
id_set = set()
person_set=set()
robot_moving=False
chosen_row = 0
nav_x=0
nav_y =0
class Person:
	def __init__(self, id, xmin,xmax,ymin,ymax):
		self.id = id
		self.xmin = xmin
		self.xmax = xmax
		self.ymin= ymin
		self.ymax= ymax
		self.x=((self.xmin+self.xmax)/2)-4
		self.y=(self.ymin+self.ymax)/2
		self.euc=0
		self.breach=False
		self.mylist = []
	
	def breaching (self, other):
		dist_rob=4
		x=((self.xmin+self.xmax)/2)-4
		y=(self.ymin+self.ymax)/2
		x_other=((other.xmin+other.xmax)/2)-4
		y_other=(other.ymin+other.ymax)/2
		euc_distance=math.sqrt(((x_other-x)**2)+((y_other-y)**2))
		self.euc=euc_distance
		if (euc_distance<1.5):
			self.breach=True
			other.breach=True
			self.mylist.append(self)
			self.mylist.append(other) 
			other.mylist.append(self)
			other.mylist.append(other)
			in_first = set(self.mylist)
			in_second = set(other.mylist)
			in_second_but_not_in_first = in_second - in_first
			self.mylist = self.mylist + list(in_second_but_not_in_first)


def get_parameters():
	"""
	Gets the necessary parameters from .yaml file
	Returns tuple
	"""
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	cost_threhold = rospy.get_param('~cost_threhold')
	min_hits = rospy.get_param('~min_hits')
	max_age = rospy.get_param('~max_age')
	return (camera_topic, detection_topic, tracker_topic, cost_threhold, max_age, min_hits)

def checkForBreach(new_person,df):
	for ind in df.index:
		#print ('comparing ID: ',new_person.id, 'and ID: ', df['Person'][ind].id)
		new_person.breaching(df['Person'][ind])
	return new_person.breach

def getCoord(breachesList):
	sum_x=0
	sum_y=0
	count=len(breachesList)
	
	for i in breachesList:
		sum_x+=i.x
		sum_y+=i.y
	return sum_x/count, sum_y/count
	
def gather_breaches(person_set):
	global nav_x
	global nav_y #x and y which are used by the robot for navigation
	global robot_moving
	global df_breaches
	global chosen_row
	for person in person_set:
		x,y=getCoord(person.mylist)
		new_row = {'Person_Breaching':person.mylist, 'nb_breaches': len(person.mylist), 'XCoord':x,'YCoord':y}
		df_breaches = df_breaches.append(new_row, ignore_index=True)
	print (df_breaches)
	row = df_breaches.loc[df_breaches['nb_breaches'].astype(float).idxmax()]
	if (robot_moving==False):
		robot_moving=True
		chosen_row = row
		x_val=row.iloc[2]
		y_val=row.iloc[3]
		nav_x=x_val
		nav_y=y_val
		return x_val,y_val
	else:
		return 0,0

#Check if the new coordinates are still close to the initial ones,
#if not then we need to send new coordintes for the robot as the pedestraisn detected probably moved
def check_coord(x_val,y_val):
	global nav_x,nav_y
	if not ((nav_x-0.5<=x_val<=nav_x+0.5) and (nav_y-0.5<=x_val<=nav_y+0.5)): 
		nav_x=x_val
		nav_y=y_val
	#send the new coordinates to the robot


def callback_det(data):
#Set the index to become the ‘ID’ column
#	df.set_index('ID') #local variable 'df' referenced before assignment
	global id_set
	global df
	global person_set
	global detections
	global trackers
	global track
	breach_bool=False
	detections = []
	trackers = []
	track = []
	bboxes = []
	global robot_moving 
	computeboxes=ComputeBoxes()
	computebox=ComputeBox()

	for box in data.bounding_boxes:
		detections.append(np.array([box.xmin_2d, box.ymin_2d, box.xmax_2d, box.ymax_2d, round(box.probability,2)]))
		detections = np.array(detections)
		trackers = tracker.update(detections)
		trackers = np.array(trackers, dtype=np.int32)
		# if (trackers.size!=0):
		# 	computebox.Class=box.Class
		# 	computebox.id=trackers[0][4]
		# 	computebox.probability=box.probability
		# 	computebox.xmin=box.xmin
		# 	computebox.ymin=box.ymin
		# 	computebox.xmax=box.xmax
		# 	computebox.ymax=box.ymax
		# 	computeboxes.compute_box.append(computebox)	
		# print('tackers: ',trackers)
		# print('new ID',trackers[0][4])
		if (trackers.size!=0):
			# xCol=df.Xmax
			# yCol=df.Ymax
			# if not (xCol.between(box.xmax-0.5,box.xmax+0.5).any() and yCol.between(box.ymax-0.25,box.ymax+0.25).any()):
			if ((trackers[0][4] not in id_set) and (len(id_set) !=0 )): #can we search directly in the df
				id_set.add(trackers[0][4])
				new_person= Person(trackers[0][4],box.xmin,box.xmax,box.ymin,box.ymax)
				person_set.add(new_person)
				#fill in the object 
				#check if there is a breach with any of the other elements
				if (checkForBreach(new_person,df)):
					breach_bool = True
					print("breach detected at ID: ",trackers[0][4])
					for ind in df.index:
						#print ('comparing ID: ',new_person.id, 'and ID: ', df['Person'][ind].id)
						print('ID: ',df['Person'][ind].id, 'Breach: ', df['Person'][ind].breach)
				new_row = {'ID':trackers[0][4],'Person':new_person, 'Xmax':new_person.xmax, 'Xmin':new_person.xmin, 'Ymax':new_person.ymax, 'Ymin':new_person.ymin}
				df = df.append(new_row, ignore_index=True)
				print(df)
			elif ((trackers[0][4] not in id_set) and (len(id_set) ==0 )):
				id_set.add(trackers[0][4])
				new_person= Person(trackers[0][4],box.xmin,box.xmax,box.ymin,box.ymax)
				person_set.add(new_person)
				new_row = {'ID':trackers[0][4],'Person':new_person, 'Xmax':new_person.xmax, 'Xmin':new_person.xmin, 'Ymax':new_person.ymax, 'Ymin':new_person.ymin}
				df = df.append(new_row, ignore_index=True)
				print(df)
		detections = []
		trackers = []
		computebox=ComputeBox()
	if (breach_bool):
		x_val,y_val=gather_breaches(person_set)
		#we need to make sure if the coordinates are changing or not 
		check_coord(x_val,y_val)

	# pub_trackers = rospy.Publisher('sort_track', ComputeBoxes,queue_size=10)
	# pub_trackers.publish(computeboxes)

def publish(trackers,box):
	computeboxes=ComputeBoxes()
	computebox=ComputeBox()
	if (trackers.size!=0):
		computebox.Class=box.Class
		computebox.id=trackers[0][4]
		computebox.probability=box.probability
		computebox.xmin=box.xmin
		computebox.ymin=box.ymin
		computebox.xmax=box.xmax
		computebox.ymax=box.ymax
		computeboxes.compute_box.append(computebox)	
	pub_trackers = rospy.Publisher('sort_track', ComputeBoxes,queue_size=10)
	pub_trackers.publish(computeboxes)

def callback_image(data):
	#Display Image
	bridge = CvBridge()
	cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
	#TO DO: FIND BETTER AND MORE ACCURATE WAY TO SHOW BOUNDING BOXES!!
	#Detection bounding box
	cv2.rectangle(cv_rgb, (int(detections[0][0]), int(detections[0][1])), (int(detections[0][2]), int(detections[0][3])), (100, 255, 50), 1)
	cv2.putText(cv_rgb , "person", (int(detections[0][0]), int(detections[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)	
	#Tracker bounding box
	cv2.rectangle(cv_rgb, (track[0][0], track[0][1]), (track[0][2], track[0][3]), (255, 255, 255), 1)
	cv2.putText(cv_rgb , str(track[0][4]), (track[0][2], track[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
	cv2.imshow("YOLO+SORT", cv_rgb)
	#print('cb1')
	cv2.waitKey(3)


def main():
	global tracker
	global msg
	msg = numpy_msg(IntList)()

	while not rospy.is_shutdown():
		#Initialize ROS node
		rospy.init_node('sort_tracker', anonymous=False)
		rate = rospy.Rate(10)
		# Get the parameters
		(camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
		tracker = sort.Sort(max_age=max_age, min_hits=min_hits) #create instance of the SORT tracker
		cost_threshold = cost_threshold
		#Subscribe to image topic
		#image_sub = rospy.Subscriber(camera_topic,Image,callback_image)
		#Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
		print('going in')
		#sub_detection = rospy.Subscriber(detection_topic, BoundingBoxes , callback_det)
		sub_detection = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d , callback_det)
		# sub_detection = message_filters.Subscriber(detection_topic, BoundingBoxes)
		# sub_3Dbbox = message_filters.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d)
		# ats = ApproximateTimeSynchronizer([sub_detection, sub_3Dbbox], queue_size=5, slop=0.1)
		# ats.registerCallback(callback_sync)

		#Publish results of object tracking
		# pub_trackers = rospy.Publisher(tracker_topic, IntList, queue_size=10)
		# #print(msg) #Testing msg that is published
		# pub_trackers.publish(msg)
		rate.sleep()
		rospy.spin()


if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass
