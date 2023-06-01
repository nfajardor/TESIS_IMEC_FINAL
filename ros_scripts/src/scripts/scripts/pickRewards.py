#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
import random
import math
from mapCreator import MapCreator
import copy

class PickRewards:
    def map_selection(self):
        map_creator = MapCreator()
        inMap = True
        while inMap:
            print("Map Selection:\n1 - Create a new map\n2 - Select existing map\n3 - Go back")
            option = input()
            print("_______________________________")
            if option == '1':
                map_creator.create_new_map()
            elif option == '2':
                self.map, self.map_name = map_creator.get_existing_map()
                if self.map != None:
                    print("The map {} was selected".format(self.map_name))
                    for i in range(0,len(self.map)):
                        print(self.map[i])
                        inMap = False
                else:
                    print("No map selected")
            elif option == '3':
                print('Exiting the map menu')
                inMap = False
            else:
                print('Please input a valid option')
    def robot_configuration_selection(self):
        map_creator = MapCreator()
        inMap = True
        while inMap:
            print("Robot Configuration:\n1 - Create a New Configuration\n2 - Use existing configuration\n3 - Exit")
            option = input()
            print("_______________________________")
            if option == '1':
                map_creator.create_robot_configuration(self.map_name)
            elif option == '2':
                self.bot_configuration = map_creator.select_robot_configuration(self.map_name)
                if self.bot_configuration != None:
                    inMap = False
                else:
                    print("No robot configuration selected")
            elif option == '3':
                inMap = False
            else:
                print("Please input a valid option")

    def set_up_task(self):
        #Get map
        self.map_selection()
        #Get robot configuration
        self.robot_configuration_selection()
        #Get reward configuration
        #Set each bot Target
        #Set each bot path
        #Translate the path to a route
        #Set the instructions for the robots


    def __init__(self):
        self.route = "./../data/"
        self.map = None
        self.filled_map = None
        self.map_name = ""
        self.bot_configuration = None
        print("The pick rewards task consists of giving instructions to a group of robots to find and 'pick-up' some rewards on the environment.")
        self.set_up_task()

