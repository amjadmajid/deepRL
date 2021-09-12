#!/usr/bin/env python3

"""
ROS node to track objects using SORT and YOLOv3 (darknet_ros)
"""


from threading import currentThread

from pandas.core.frame import DataFrame
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
df_breaches = pd.DataFrame(columns=('Breach_ID','Person_Breaching', 'nb_breaches', 'XCoord','YCoord')) #Person_Breaching is a list of persons
df_persons = pd.DataFrame(columns=('PID','Person','Breach_ID'))

id_set = set()
person_set=set()
breaches_dict={}
robot_moving=False
chosen_row = 0
nav_x=0
nav_y =0
breaches_set=set()
dic_ID=1
set2=set([frozenset([1,2]), frozenset([3,4])])

class Person:
	def __init__(self, id, xmin,xmax,ymin,ymax):
		self.id = id
		self.xmin = xmin
		self.xmax = xmax
		self.ymin= ymin
		self.ymax= ymax
		self.x=((self.xmin+self.xmax)/2)-4
		#self.x=((self.xmin+self.xmax)/2)
		self.y=(self.ymin+self.ymax)/2
		self.euc=0
		self.breach=False
	
	def breaching (self, other,df):
		dist_rob=4
		global dic_ID
		x=((self.xmin+self.xmax)/2)-4
		#x=((self.xmin+self.xmax)/2)
		y=(self.ymin+self.ymax)/2
		x_other=((other.xmin+other.xmax)/2)-4
		#x_other=((other.xmin+other.xmax)/2)
		y_other=(other.ymin+other.ymax)/2
		euc_distance=math.sqrt(((x_other-x)**2)+((y_other-y)**2))
		self.euc=euc_distance
		if (euc_distance<1.5 and self.id != other.id):
			self.breach=True
			other.breach=True
			temp1=df.iat[self.id,2]
			temp2=df.iat[other.id,2]
			if (temp1 ==0 and temp2==0):
				temp = set()
				temp.add(self.id)
				temp.add(other.id)
				breaches_dict[dic_ID] = temp
				df.loc[self.id, 'Breach_ID'] = dic_ID
				df.loc[other.id, 'Breach_ID'] = dic_ID
				dic_ID+=1
			elif (temp1 ==0 and temp2!=0):
				temp = set()
				temp.add(self.id)
				temp.add(other.id)
				temp_list=temp.union(breaches_dict.get(temp2))
				breaches_dict[dic_ID] = temp_list
				# breaches_dict.pop(temp2)
				df.loc[self.id, 'Breach_ID'] = dic_ID
				df.loc[other.id, 'Breach_ID'] = dic_ID	
				dic_ID+=1			
			else:
				temp_list=breaches_dict.get(temp1).union(breaches_dict.get(temp2))
				breaches_dict[temp1] = temp_list
				#breaches_dict.pop(temp2)
				df.loc[other.id, 'Breach_ID'] = temp1


def get_parameters():
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	cost_threhold = rospy.get_param('~cost_threhold')
	min_hits = rospy.get_param('~min_hits')
	max_age = rospy.get_param('~max_age')
	return (camera_topic, detection_topic, tracker_topic, cost_threhold, max_age, min_hits)

def checkForBreach(new_person,df):
	for ind in df.index:
		new_person.breaching(df['Person'][ind],df)
	return new_person.breach

def getCoord(breachesList):
	global df_persons
	sum_x=0
	sum_y=0
	count=len(breachesList)
	#print (breachesList)
	for i in breachesList:
		sum_x+=df_persons.iat[i,1].x
		sum_y+=df_persons.iat[i,1].y
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
		print('here',nav_x )
		return nav_x,nav_y
	else:
		return nav_x,nav_y

#Check if the new coordinates are still close to the initial ones,
#if not then we need to send new coordintes for the robot as the pedestraisn detected probably moved
def check_coord(x_val,y_val):
	global nav_x,nav_y
	if not ((nav_x-0.5<=x_val<=nav_x+0.5) and (nav_y-0.5<=x_val<=nav_y+0.5)): 
		nav_x=x_val
		nav_y=y_val


def callback_det(data):
	global id_set
	global df
	global detections
	global trackers
	global track
	global df_persons
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

		if (trackers.size!=0):
			new_ID=trackers[0][4]-1
			if ((new_ID not in id_set)): #can we search directly in the df
				
				id_set.add(new_ID)
				new_person= Person(new_ID,box.xmin,box.xmax,box.ymin,box.ymax)
				new_row = {'PID':new_ID,'Person':new_person,'Breach_ID':0}  #do it as a map
				df_persons = df_persons.append(new_row, ignore_index=True)
				if ((len(id_set) >1 ) and checkForBreach(new_person,df_persons)):
					breach_bool = True
					# print("breach detected at ID: ",new_ID)
				print(df_persons)
			
		detections = []
		trackers = []
		computebox=ComputeBox()
		#print(df_persons)
	if (breach_bool):
		x_val,y_val=gather_breaches()
		print(df_persons)
		print(breaches_dict)
		#we need to make sure if the coordinates are changing or not 
		check_coord(x_val,y_val)
		breach_bool=False
		pub = rospy.Publisher('findObjects', Object, queue_size=1)
		msg = Object()
		msg.x= x_val
		msg.y= y_val
		msg.z= 0
		pub.publish(msg)

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
		print('going in')
		sub_detection = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d , callback_det)
		rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass
