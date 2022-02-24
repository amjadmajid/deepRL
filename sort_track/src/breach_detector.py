#!/usr/bin/env python3
import argparse
import rospy
import torch
import numpy as np
import pandas as pd
import csv
import PIL
from sensor_msgs import msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openpifpaf import datasets
from openpifpaf import decoder, network, visualizer, show, logger, Predictor
from openpifpaf.predict import main, out_name
from monoloco.predict import get_torch_checkpoints_dir,download_checkpoints, factory_from_args, factory_outputs
from monoloco.visuals import Printer
from monoloco.network import net, Loco,preprocess_pifpaf, load_calibration
from monoloco.run import cli
from PIL import Image as PIL_Image
import time
import logging
import json
import os
import cv2
import numpy as np

LOG = logging.getLogger(__name__)

class DetectorManager():
    def __init__(self):

        self.image_topic = rospy.get_param('~image_topic', '/realsense_d435/color/image_raw')

        self.argString = rospy.get_param('~monoloco_args', 'predict docs/frame0032.jpg --activities social_distance --output_types front bird --n_dropout 50')
        self.x_list_1=[]
        self.y_list_1=[]
        self.counter=0
        rospy.loginfo('People detection starting')
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size = 1, buff_size = 2**24)
        rospy.spin()

    def imageCb(self,data):

        self.cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        cv2.imwrite('camera_image.jpeg', self.cv_image)
        os.system('python3 -m monoloco.run predict camera_image.jpeg --activities social_distance --output_types json --n_dropout 50')
        f = open('out_camera_image.jpeg.monoloco.json')
        data_json = json.load(f)
        status=data_json["social_distance"]
        resp = data_json["xyz_pred"]
        print(resp)
        ale=data_json["stds_ale"]
        ep=data_json["stds_epi"]

        if (len(resp)>=1):
            # print(resp)
            x1=resp[0][2]
            y1=resp[0][0]
            self.x_list_1.append(x1)
            self.y_list_1.append(y1)
            self.counter+=1
        print(self.counter)
        if (self.counter==50):
            print("here")
            df = pd.DataFrame(data={"x1": self.x_list_1, "y1": self.y_list_1})
            file_name = '/home/serge/Desktop/DD_ws/excel1/127.xlsx'
            df.to_excel(file_name)


if __name__=="__main__":
    rospy.init_node("detector_manager_node")
    dm = DetectorManager()
