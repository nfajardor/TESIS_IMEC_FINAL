#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
import random
import math

class PickRewards:
    def __init__(self):
        self.route = "./../data/"
        print("About to pick rewards")
    