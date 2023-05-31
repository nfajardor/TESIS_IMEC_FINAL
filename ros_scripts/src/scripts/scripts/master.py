#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
import random
import math
from mapCreator import MapCreator

if __name__ == '__main__':
    exitApp = False
    map_creator = MapCreator()
    while not exitApp:
        print("Please select what to do:\n1 - Map editor\n2 - Do a task\n3 - Exit the App")
        option = input()
        print("_______________________________")
        if option == '1':
            inMap = True
            while inMap:
                print("Map Menu:\n1 - Create a new map\n2 - Edit existing map\n3 - Go back")
                option = input()
                print("_______________________________")
                if option == '1':
                    map_creator.create_new_map()
                elif option == '2':
                    map_creator.edit_previous_map()
                elif option == '3':
                    print('Exiting the map menu')
                    inMap = False
                else:
                    print('Please input a valid option')

        elif option == '2':
            print("Task Selection")
        elif option == '3':
            print("Exiting the app")
            exitApp = True
        else:
            print("Please select a valid option")
    print("Successfully exited the app")
