#!/usr/bin/env python3
import argparse
import rospy
import torch
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import Object
import numpy as np
# from darknet_ros_msgs.msg import Coordinates
import time
import logging
import json
import os
import cv2

from respawnGoal import Respawn

class Detectager():
    def __init__(self):
        self.respawn_goal = Respawn()
