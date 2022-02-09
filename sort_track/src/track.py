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
# from darknet_ros_msgs.msg import Object
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
x_list_1=[]
x_list_2=[]
y_list_1=[]
y_list_2=[]
breaches_list=[]
final_counter=0

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
	for key in person_dict:#Check if the identified person is breaching social distancing with all people already identified
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
		
	# print("going towards breach: ",key)
	x,y=getCoord(breaches_dict.get(max_key))
	# print("x,y: ",nav_x,nav_y)
	if (robot_moving==False):
		robot_moving=True
		nav_x=x
		nav_y=y
	return nav_x,nav_y
	# else:1.91
	# if not ((nav_x-0.5<=x_val<=nav_x+0.5) and (nav_y-0.5<=x_val<=nav_y+0.5)): 
	# 	nav_x=x_val
	# 	nav_y=y_val


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
	global final_counter
	breach_bool=False
	track_bool=False
	detections = []
	trackers = []
	track = []
	bboxes = []
	global x_list_1
	global y_list_1
	global x_list_2
	global y_list_2
	global breaches_list
	computeboxes=ComputeBoxes()
	computebox=ComputeBox()

	msg1 = Object()
	msg1.x= 0
	msg1.y= 0
	msg1.z= 0
	pub.publish(msg1)
	
	#Loops through the identified bounding boxes
	for box in data.bounding_boxes:
		
		#Use the SORT algorithm to track the people identified
		detections.append(np.array([box.xmin_2d, box.ymin_2d, box.xmax_2d, box.ymax_2d, round(box.probability,2)]))
		detections = np.array(detections)
		trackers = tracker.update(detections)
		trackers = np.array(trackers, dtype=np.int32)
		# print("new bbox")
		if (trackers.size!=0): #If people were detected
			
			new_ID=trackers[0][4]-1 #Get the ID of the detected person
			# print(new_ID)
			# print("new person detected, ID: ", new_ID)
			if ((new_ID not in id_set)): #The person is identified for the first time
				for key, value in person_dict.items(): #Make sure that this is the first time this person is identified by looking into the dict of people
					if ((value.xmin-0.2<box.xmin<value.xmin+0.2) and (value.xmax-0.2<box.xmax<value.xmax+0.2) and (value.ymin-0.2<box.ymin<value.ymin+0.2) and (value.ymax-0.2<box.ymax<value.ymax+0.2)):
						track_bool=True
						new_ID=key
						new_person=person_dict.get(new_ID) #if this person is already tracked get his information
						new_person.updateCoord(box.xmin,box.xmax,box.ymin,box.ymax)
						new_person.addCoord(new_person.coord)
						#average_coord.get(new_ID).append(new_person.coordinates) #Add the coordinates to the coordinates dic to later get the average
						break
				if not (track_bool):
					id_set.add(new_ID) #Add the ID to the set of IDs
					new_person= Person(new_ID,box.xmin,box.xmax,box.ymin,box.ymax)
					new_person.addCoord(new_person.coord)
					person_dict[new_ID] = new_person #insert the information about the newly tracked person
					#average_coord[new_ID]=[new_person.coordinates] #Add the coordinates to the coordinates dic to later get the average
			else:
				new_person=person_dict.get(new_ID) #if this person is already tracked get his information
				new_person.updateCoord(box.xmin,box.xmax,box.ymin,box.ymax)
				new_person.addCoord(new_person.coord)				
				#average_coord.get(new_ID).append(new_person.coordinates) #Add the coordinates to the coordinates dic to later get the average
			x_list_1.append(new_person.x)
			y_list_1.append(new_person.y)
			if ((len(id_set) >1 ) and checkForBreach(new_person)):
				breach_bool = True
		track_bool=False	
		detections = []
		trackers = []

	if (counter==1):
		counter=0
		x_val=0
		y_val=0
		if (breach_bool):
			x_val,y_val=gather_breaches()
			breaches_list.append("True")
			print("True")
			#we need to make sure if the coordinates are changing or not 
			# check_coord(x_val,y_val)
			breach_bool=False
		else:
			breaches_list.append("False")
			
		pub = rospy.Publisher('findObjects', Object, queue_size=1)
		print(pd.DataFrame(person_dict.items()))
		print(pd.DataFrame(breaches_dict.items()) )
		# if (len(breaches_dict)>0):
		# 	print('***\n---------------------- Finished Tracking -----------------------\n****')
		# 	print ("Number of people identified: " , len(person_dict))
		# 	print(pd.DataFrame(person_dict.items()))
		# 	print("Number of breaches identified: ", len(breaches_dict))
		# 	print(pd.DataFrame(breaches_dict.items()) )
		# 	print("Going towards the coordinates x and y: ", x_val, y_val )
		# 	print('***\n---------------------- New Tracking -----------------------\n****')
		# print('publishing',x_val,y_val)
		msg = Object()
		msg.x= x_val
		msg.y= y_val
		msg.z= 0
		pub.publish(msg)
		# tracker.reset_count()
		person_dict={}
		BID_dict = {}
		breaches_dict={}
		average_coord={}
		dic_ID=1
		id_set = set()


	else:
		counter=counter+1
	print(len(x_list_1))
	if (len(x_list_1)==100):
		# df=pd.DataFrame(data={"x1":x_list_1,"y1":y_list_1})
		# file_name = '/home/jetbot/Desktop/camera_ws/78.xlsx'
		# df.to_excel(file_name)
		print("Done measurements")
	# print(len(breaches_list))
	# if (len(breaches_list)==100):
	# 	df=pd.DataFrame(data={"Status":breaches_list})
	# 	file_name1 = '/home/jetbot/Desktop/camera_ws/status/30.xlsx'
	# 	df.to_excel(file_name1)
	# 	print('***\n---------------------- New Tracking ------*************-----------------\n****')	


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
	global robot_moving
	msg = numpy_msg(IntList)()




	while not rospy.is_shutdown():
		#Initialize ROS node
		rospy.init_node('sort_tracker', anonymous=False)
		rate = rospy.Rate(10)
		msg1 = Object()
		msg1.x= 0
		msg1.y= 0
		msg1.z= 0
		pub = rospy.Publisher('findObjects', Object, queue_size=1)
		pub.publish(msg1)
		# Get the parameters
		(camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
		tracker = sort.Sort(max_age=max_age, min_hits=min_hits) #create instance of the SORT tracker
		cost_threshold = cost_threshold
		print('going in')
		sub_detection = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d , callback_det)
		rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass