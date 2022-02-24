#!/usr/bin/env python3

"""
ROS node to track objects using SORT and YOLOv3 (darknet_ros)
"""
from threading import currentThread
from pandas.core.frame import DataFrame
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ComputeBoxes,ComputeBox
from sort import sort 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sort_track.msg import IntList
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64MultiArray
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from message_filters import ApproximateTimeSynchronizer, Subscriber
from custom_msgs.msg import Object
import message_filters
import math 
import pandas as pd
import rospy
import numpy as np
import cv2

person_dict={}
BID_dict = {}
breaches_dict={}
average_coord={}
id_set = set()
counter=0
robot_moving=False
nav_x=0
nav_y =0
dic_ID=1

class Coord:
	def __init__(self, x,y):
		self.x=x
		self.y=y

class Person:
	def __init__(self, id, xmin,xmax,ymin,ymax):
		self.id = id
		self.xmin = xmin
		self.xmax = xmax
		self.ymin= ymin
		self.ymax= ymax
		self.x=((self.xmin+self.xmax)/2)
		self.y=(self.ymin+self.ymax)/2
		self.euc=0
		self.breach=False
		self.coordList=[]
		self.coord=Coord(self.x,self.y)
		self.x_average=0
		self.y_average=0

	def updateCoord(self,xmin,xmax,ymin,ymax):
		self.xmin = xmin
		self.xmax = xmax
		self.ymin= ymin
		self.ymax= ymax
		self.x=((self.xmin+self.xmax)/2)
		self.y=(self.ymin+self.ymax)/2	
		self.coord=Coord(self.x,self.y)

	def addCoord(self,coord):
		self.coordList.append(coord)

	def computeCoord (self):
		count=len(self.coordList)
		self.x_average=0
		self.y_average=0
		for coordinate in self.coordList:
			self.x_average+=coordinate.x
			self.y_average+=coordinate.y
		self.x_average=self.x_average/count
		self.y_average=self.y_average/count
		self.coordList=[]

	def breaching (self, key):
		dist_rob=4
		global dic_ID
		global person_dict
		global BID_dict 
		x=((self.xmin+self.xmax)/2)
		y=(self.ymin+self.ymax)/2

		other=person_dict.get(key)
		x_other=((other.xmin+other.xmax)/2)
		y_other=(other.ymin+other.ymax)/2
		
		euc_distance=math.sqrt(((x_other-x)**2)+((y_other-y)**2))
		self.euc=euc_distance
		if (euc_distance<1 and self.id != other.id):
			self.breach=True
			other.breach=True
			if not(self.id in BID_dict and key in BID_dict):
				temp = set()
				temp.add(self.id)
				temp.add(other.id)
				breaches_dict[dic_ID] = temp
				BID_dict[self.id] = dic_ID
				BID_dict[key] = dic_ID
				dic_ID+=1
			elif (self.id not in BID_dict and key in BID_dict):
				temp = set()
				temp.add(self.id)
				temp.add(other.id)
				temp2=BID_dict.get(key)
				temp_list=temp.union(breaches_dict.get(temp2))
				breaches_dict[temp2] = temp_list
				# breaches_dict.pop(temp2)
				# BID_dict[self.id] = dic_ID
				BID_dict[key] = temp2	
				# dic_ID+=1	
			elif (BID_dict.get(self.id)==BID_dict.get(key)):
				pass
			else:
				temp1=BID_dict.get(self.id)
				temp2=BID_dict.get(key)
				temp_list=breaches_dict.get(temp1).union(breaches_dict.get(temp2))
				breaches_dict[temp1] = temp_list
				#breaches_dict.pop(temp2)
				BID_dict[key] = temp1


def get_parameters():
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	cost_threhold = rospy.get_param('~cost_threhold')
	min_hits = rospy.get_param('~min_hits')
	max_age = rospy.get_param('~max_age')
	return (camera_topic, detection_topic, tracker_topic, cost_threhold, max_age, min_hits)

def checkForBreach(new_person):
	global person_dict
	global BID_dict 
	for key in person_dict: #Check if the identified person is breaching social distancing with all people already identified
		new_person.breaching(key)
	return new_person.breach

def getCoord(breachesList):
	global person_dict
	global BID_dict 
	sum_x=0
	sum_y=0
	count=len(breachesList)
	for i in breachesList:
		person_dict.get(i).computeCoord()
		sum_x+=person_dict.get(i).x_average
		sum_y+=person_dict.get(i).y_average
	return sum_x/count, sum_y/count
	
def gather_breaches():
	global nav_x
	global nav_y #x and y which are used by the robot for navigation
	global robot_moving
	global breaches_dict
	count=0
	max_key=0
	for key, value in breaches_dict.items():
		
		if (count==0):
			max=len(value)
			max_key=key
		elif max<len(value):
			max=len(value)
			max_key=key
		
	x,y=getCoord(breaches_dict.get(max_key))
	if (robot_moving==False):
		robot_moving=True
		nav_x=x
		nav_y=y
	return nav_x,nav_y


def callback_det(data):
	global id_set
	global df
	global detections
	global trackers
	global track
	global person_dict
	global BID_dict 
	global robot_moving 
	global counter
	global average_coord
	global dic_ID
	global breaches_dict

	breach_bool=False
	track_bool=False
	detections = []
	trackers = []
	track = []
	bboxes = []
	computeboxes=ComputeBoxes()
	computebox=ComputeBox()
	
	for box in data.bounding_boxes:
		detections.append(np.array([box.xmin_2d, box.ymin_2d, box.xmax_2d, box.ymax_2d, round(box.probability,2)]))
		detections = np.array(detections)
		trackers = tracker.update(detections)
		trackers = np.array(trackers, dtype=np.int32)
		if (trackers.size!=0): #If people were detected
			new_ID=trackers[0][4]-1
			if ((new_ID not in id_set)): #new person detected
				# make sure it is a new person by comparing its coordinates to all previously identified people 
				for key, value in person_dict.items():
					if ((value.xmin-0.2<box.xmin<value.xmin+0.2) and (value.xmax-0.2<box.xmax<value.xmax+0.2) and (value.ymin-0.2<box.ymin<value.ymin+0.2) and (value.ymax-0.2<box.ymax<value.ymax+0.2)):
						track_bool=True
						new_ID=key
						new_person=person_dict.get(new_ID)
						new_person.updateCoord(box.xmin,box.xmax,box.ymin,box.ymax)
						new_person.addCoord(new_person.coord)
						#average_coord.get(new_ID).append(new_person.coordinates) #Add the coordinates to the coordinates dic to later get the average
						break
				if not (track_bool): #If this person is really identified for the first time
					id_set.add(new_ID)
					new_person= Person(new_ID,box.xmin,box.xmax,box.ymin,box.ymax)
					new_person.addCoord(new_person.coord)
					person_dict[new_ID] = new_person
					#average_coord[new_ID]=[new_person.coordinates] #Add the coordinates to the coordinates dic to later get the average
			else:
				new_person=person_dict.get(new_ID) #if this person is already tracked get his information
				new_person.updateCoord(box.xmin,box.xmax,box.ymin,box.ymax)
				new_person.addCoord(new_person.coord)				
				#average_coord.get(new_ID).append(new_person.coordinates) #Add the coordinates to the coordinates dic to later get the average
			if ((len(id_set) >1 ) and checkForBreach(new_person)):
				breach_bool = True
		track_bool=False	
		detections = []
		trackers = []

	if (counter==20):
		counter=0
		x_val=0
		y_val=0
		if (breach_bool):
			x_val,y_val=gather_breaches()
			breach_bool=False
			
		pub = rospy.Publisher('findObjects', Object, queue_size=1)
		msg = Object()
		msg.x= x_val
		msg.y= y_val
		msg.z= 0
		pub.publish(msg)
		
		person_dict={}
		BID_dict = {}
		breaches_dict={}
		average_coord={}
		dic_ID=1
		id_set = set()
	else:
		counter=counter+1


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

def main():
	global tracker
	global msg
	global robot_moving
	msg = numpy_msg(IntList)()
	while not rospy.is_shutdown():
		rospy.init_node('sort_tracker', anonymous=False)
		rate = rospy.Rate(10)
		msg1 = Object()
		msg1.x= 0
		msg1.y= 0
		msg1.z= 0
		pub = rospy.Publisher('findObjects', Object, queue_size=1)
		pub.publish(msg1)
		(camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
		tracker = sort.Sort(max_age=max_age, min_hits=min_hits) #create instance of the SORT tracker
		cost_threshold = cost_threshold
		sub_detection = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d , callback_det)
		rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass