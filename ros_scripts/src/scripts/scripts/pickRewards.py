#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String
import random
import math
from mapCreator import MapCreator
import copy

class PickRewards:

    #Method to select the map to use for the task
    def map_selection(self):
        map_creator = MapCreator()
        inMap = True
        while inMap:
            print("Map Selection:\n1 - Create a new map\n2 - Select existing map\n3 - Go back")
            option = input()
            print("_______________________________")

            #Opens the creation menu for a new map
            if option == '1':
                map_creator.create_new_map()

            #Opens a selection menu to select one of the already existing maps
            elif option == '2':
                self.map, self.map_name = map_creator.get_existing_map()
                if self.map != None:

                    #Prints the selected map in the console
                    print("The map {} was selected".format(self.map_name))
                    for i in range(0,len(self.map)):
                        print(self.map[i])
                        inMap = False
                    
                    #Set the bot and rwd map as copies of the original
                    self.bot_map = copy.deepcopy(self.map)
                    self.rwd_map = copy.deepcopy(self.map)
                    
                    #Get the width, height and sizew of the map
                    self.height = len(self.map)
                    self.width = len(self.map[0])
                    self.size = self.width*self.height

                    #Sets a negative map used for targeting and path planning 
                    self.neg_map = []
                    for i in range(0,len(self.map)):
                        self.neg_map.append([])
                        for j in self.map[i]:
                            if j == 0:
                                self.neg_map[i].append(self.size)
                            else:
                                self.neg_map[i].append(-j)
                    
                else:
                    print("No map selected")
            elif option == '3':
                print('Exiting the map menu')
                inMap = False
            else:
                print('Please input a valid option')


    #Method to select the robot configuration to use for the task
    def robot_configuration_selection(self):
        map_creator = MapCreator()
        inMap = True
        while inMap:
            print("Robot Configuration:\n1 - Create a New Configuration\n2 - Use existing configuration\n3 - Exit")
            option = input()
            print("_______________________________")

            #Opens menu to create a new robot configuration
            if option == '1':
                map_creator.create_robot_configuration(self.map_name)

            #Opens menu to select an existing configuration
            elif option == '2':

                #Gets the configuration
                self.bot_configuration = map_creator.select_robot_configuration(self.map_name)
                if self.bot_configuration != None:
                    inMap = False

                    #updates the bot map with the info on the configuration
                    print("Configuration Selected: {}".format(self.bot_configuration))
                    for bc in self.bot_configuration:
                        x_coord,y_coord = bc[1][0],bc[1][1]
                        self.bot_map[y_coord][x_coord] = 3

                    #Prints the bot map
                    print("BOT map:")
                    for i in self.bot_map:
                        print(i)
                else:
                    print("No robot configuration selected")
            elif option == '3':
                inMap = False
            else:
                print("Please input a valid option")

    #Selection of the reward configuration for the task
    def reward_configuration_selection(self):
        map_creator = MapCreator()
        inMap = True
        while inMap:
            print("Reward Configuration:\n1 - Create a New Configuration\n2 - Use existing configuration\n3 - Exit")
            option = input()
            print("_______________________________")

            #Opens menu to create a new configuration
            if option == '1':
                map_creator.create_reward_configuration(self.map_name)

            #Opens menu to select an existing configuration
            elif option == '2':

                #Gets the configuration
                self.rwd_configuration = map_creator.select_reward_configuration(self.map_name)
                if self.rwd_configuration != None:
                    inMap = False

                    #Updates the reward map with the info on the configuration
                    print("Configuration Selected: {}".format(self.rwd_configuration))
                    for rc in self.rwd_configuration:
                        x_coord,y_coord = rc[0],rc[1]
                        self.rwd_map[y_coord][x_coord] += 2

                    #Prints the reward map
                    print("RWD map:")
                    for i in self.rwd_map:
                        print(i)
                else:
                    print("No robot configuration selected")
            elif option == '3':
                inMap = False
            else:
                print("Please input a valid option")

    #Sets the target rewards for each robot
    def set_bots_target(self):
        print("_______________________________")
        print("_______________________________")
        print("Setting the target rewards for each robot")
        print("_______________________________")
        print("_______________________________")
        exploration_front = []  #2D array with the next cells that the robots will explore
        bot_steps = []          #The current steps on each robot
        map_matrix = []         #3D matrix that has a neg map for each bot
        self.targeted_rewards = []
        untargeted_rwds = copy.deepcopy(self.rwd_configuration)
        print("_______________________________")
        
        #Setting the initial conditions
        for i in range(0,len(self.bot_configuration)):
            bot = self.bot_configuration[i]
            map_matrix.append(copy.deepcopy(self.neg_map))
            print("Created initial map for {}".format(bot[0]))
            exploration_front.append([bot[1]])
            print("Added {} to {}'s exploration front: {}".format(bot[1],bot[0],exploration_front[i]))
            bot_steps.append(0)
            print("{}'s step count set to {}".format(bot[0],bot_steps[i]))
            self.targeted_rewards.append([])
            print("Current targeted rewards for {} is of length {}: {}".format(bot[0],len(self.targeted_rewards[i]),self.targeted_rewards[i]))
            print("_______________________________")
        
        #Starting exploration. Stop when all rewards are targeted
        while len(untargeted_rwds) > 0:
            #Iterate over eaech robot
            for i in range(0,len(self.bot_configuration)):
                bot = self.bot_configuration[i]
                reward_added = False
                temp_cells = []
                #Iterate over each cell in the exploration front
                for j in exploration_front[i]:
                    if not reward_added:
                        x = j[0]
                        y = j[1]

                        #If the current cell being explored is an untargeted reward, it is targeted and resets temp cells (and the exp. front) to the current cell. Also resets the steps and the map matrix for the robot
                        if j in untargeted_rwds:
                            print("{} found a reward at {}!".format(bot[0],j))
                            untargeted_rwds.remove(j)

                            map_matrix[i] = copy.deepcopy(self.neg_map)
                            bot_steps[i] = 0
                            temp_cells = []
                            temp_cells.append([x,y])
                            self.targeted_rewards[i].append([x,y])
                            reward_added = True
                        
                        #If it is not an untargeted reward, update that cell's step and add all the adj unvisited cells to the exploration front
                        else:
                            step = bot_steps[i]
                            map_matrix[i][y][x] = step
                            
                            #UP
                            if y > 0:
                                if map_matrix[i][y-1][x] > step and [[x,y-1]] not in exploration_front[i] and [[x,y-1]] not in temp_cells:
                                    temp_cells.append([x,y-1])
                            #DOWN
                            if y < self.height - 1:
                                if map_matrix[i][y+1][x] > step and [[x,y+1]] not in exploration_front[i] and [[x,y+1]] not in temp_cells:
                                    temp_cells.append([x,y+1])
                            #RIGHT
                            if x < self.width - 1:
                                if map_matrix[i][y][x+1] > step and [[x+1,y]] not in exploration_front[i] and [[x+1,y]] not in temp_cells:
                                    temp_cells.append([x+1,y])
                            #LEFT
                            if x > 0:
                                if map_matrix[i][y][x-1] > step and [[x-1,y]] not in exploration_front[i] and [[x-1,y]] not in temp_cells:
                                    temp_cells.append([x-1,y])
                            
                bot_steps[i] += 1
                exploration_front[i] = temp_cells
        print("All rewards Targeted!")
        for i in range(0,len(self.bot_configuration)):
            print("{} has {} targets: {}".format(self.bot_configuration[i][0],len(self.targeted_rewards[i]),self.targeted_rewards[i]))

    #Finds the path between two cells
    def find_path_between_cells(self,o,t):
        pathing_map = copy.deepcopy(self.neg_map)
        step = 0
        exp = []
        exp.append(copy.deepcopy(o))
        arrived = False
        while not arrived:
            temp = []
            for i in exp:
                x = i[0]
                y = i[1]
                #print("X is {} Y is {}".format(x,y))
                #Set the current position at the current step
                pathing_map[y][x] = step
                if i == t:
                    arrived = True
                else:

                    #add unvisited adjacent to the exp

                    #UP
                    if y > 0:
                        if pathing_map[y-1][x] > step and [[x,y-1]] not in i and [[x,y-1]] not in temp:
                            temp.append([x,y-1])
                    #DOWN
                    if y < self.height - 1:
                        if pathing_map[y+1][x] > step and [[x,y+1]] not in i and [[x,y+1]] not in temp:
                            temp.append([x,y+1])
                    #RIGHT
                    if x < self.width - 1:
                        if pathing_map[y][x+1] > step and [[x+1,y]] not in i and [[x+1,y]] not in temp:
                            temp.append([x+1,y])
                    #LEFT
                    if x > 0:
                        if pathing_map[y][x-1] > step and [[x-1,y]] not in i and [[x-1,y]] not in temp:
                            temp.append([x-1,y])
            step += 1
            exp = temp
        #print("Path from {} to {} found in step {}:".format(o,t, step))
        path = []
        cur_pos = copy.deepcopy(t)
        cur_step = pathing_map[t[1]][t[0]]
        while len(path) < step:
            #Add the current position to the path
            x = cur_pos[0]
            y = cur_pos[1]
            path.append([x,y])
            
            #Set the current position as the lowest step
            new_x = x
            new_y = y
            found = False
            #UP
            if y > 0 and not found:
                if pathing_map[y-1][x] < pathing_map[y][x] and pathing_map[y-1][x] >= 0:
                    new_y = y - 1
                    found = True
            #DOWN
            if y < self.height - 1 and not found:
                if pathing_map[y+1][x] < pathing_map[y][x] and pathing_map[y+1][x] >= 0:
                    new_y = y + 1
                    found = True
            #RIGHT
            if x < self.width - 1 and not found:
                if pathing_map[y][x+1] < pathing_map[y][x] and pathing_map[y][x+1] >= 0:
                    new_x = x + 1
                    found = True
            #LEFT
            if x > 0 and not found:
                if pathing_map[y][x-1] < pathing_map[y][x] and pathing_map[y][x-1] >= 0:
                    new_x = x - 1
                    found = True
            cur_pos[0] = new_x
            cur_pos[1] = new_y

        #print("Path is: {}".format(path[::-1]))
        return path[::-1]    


        


    #Set the path reward for each robot
    def set_bots_path(self):
        print("_______________________________")
        print("_______________________________")
        print("Setting paths")
        print("_______________________________")
        print("_______________________________")
        self.bot_paths = []
        for i in range(0,len(self.bot_configuration)):
            self.bot_paths.append([])
            bot = self.bot_configuration[i]
            init_pos = copy.deepcopy(bot[1])
            self.bot_paths[i].append(init_pos)
            for j in self.targeted_rewards[i]:
                #pos = copy.deepcopy(j)
                #print("Looking for path form {} to {}".format(copy.deepcopy(init_pos),copy.deepcopy(j)))
                temp = self.find_path_between_cells(init_pos,j)
                
                for k in range(1,len(temp)):
                    self.bot_paths[i].append(temp[k])

                
                init_pos = copy.deepcopy(j)
                

            print("{}'s path is: {}".format(bot[0],self.bot_paths[i]))
            print("_______________________________")

    #Gets the direction the bot needs to go to to get from an initial cell p0 to p1
    def get_direction(self,p0,p1):
        res = [p1[0]-p0[0],p1[1]-p0[1]]
        if res[0] == 1:
            return 'E'
        elif res[0] == -1:
            return 'W'
        elif res[1] == -1:
            return 'N'
        else:
            return 'S'

    #Sets each of the bots route. In this case, route reffers to the direction the bot has to go to to get to the next cell
    def set_bots_route(self):
        print("_______________________________")
        print("_______________________________")
        print("Setting routes")
        print("_______________________________")
        print("_______________________________")
        self.bot_routes = []

        #Iterate over all the robots
        for i in range(0,len(self.bot_configuration)):
            bot = self.bot_configuration[i]
            self.bot_routes.append([])
            for j in range(0,len(self.bot_paths[i])-1):
                cur_pos = self.bot_paths[i][j]
                nex_pos = self.bot_paths[i][j+1]
                self.bot_routes[i].append(self.get_direction(cur_pos,nex_pos))
            print("{}'s route is: {}".format(bot[0],self.bot_routes[i]))

    #Gets the index on the rotation matrix of a given orientation
    def get_rotation_index(self,ori):
        if ori == 'N':
            return 0
        elif ori == 'S':
            return 1
        elif ori == 'E':
            return 2
        else:
            return 3

    #Gets the ammount of rotation required (counterclockwise)
    def get_rotation(self,o0,o1):
        j = self.get_rotation_index(o0)
        i = self.get_rotation_index(o1)
        return self.bot_rotation_matrix[j][i]

    #Checks if the next instruction should be rotation or turning
    def check_turning_or_rotation(self,initial,target):
        res = self.get_rotation(initial,target)
        if res == 2:
            return [0,2]
        elif res == 1:
            return [3,1]
        elif res == 3:
            return [3,0]
        else:
            print("ERRORERRORERRORERROR")
            return None

    #Gets the orientation vector
    def get_ori_vector(self,ori):
        if ori == 'N':
            return [0,-1]
        elif ori == 'S':
            return [0,1]
        elif ori == 'E':
            return [1,0]
        else:
            return [-1,0]
        
    #Gets the resulting orientation after n turns
    def turn_ori(self,ori,n):
        new_ori = ori
        while n>0:
            n-=1
            if new_ori == 'N':
                new_ori = 'W'
            elif new_ori == 'W':
                new_ori = 'S'
            elif new_ori == 'S':
                new_ori = 'E'
            else:
                new_ori = 'N'
        return new_ori   


    #Sets the instructions for the robot
    def set_bot_instructions(self):
        print("_______________________________")
        print("_______________________________")
        print("Setting instructions")
        print("_______________________________")
        print("_______________________________")
        self.bot_instructions = []
        
        #Iterate over all robots
        for i in range(0,len(self.bot_configuration)):
            bot = self.bot_configuration[i]
            self.bot_instructions.append([])
            ori = copy.deepcopy(bot[2])
            pos = copy.deepcopy(bot[1])
            #print("Setting the route for bot {} with pos {} and ori {}".format(bot[0],pos,ori))
            step = 0
            #While the route is not complete
            while step < (len(self.bot_routes[i])-1):
                #print("Checking step {} of robot {}: {}".format(step,bot[0],self.bot_routes[i][step]))
                cur_obj = copy.deepcopy(self.bot_routes[i][step])
                #print("Current objective orientation is {}".format(cur_obj))

                #Check if the current orientation is different from objective
                if cur_obj != ori:
                    instruction = 0
                    ammount = self.get_rotation(ori,cur_obj)
                    self.bot_instructions[i].append([instruction,ammount])
                    #print("INSTRUCTION ADDED ({},{}):\n{} added inplace rotation from {} to {}".format(instruction, ammount, bot[0],ori,cur_obj))
                    ori = cur_obj
                blocks = 0
                while ori == cur_obj:
                    blocks += 1
                    step += 1
                    if step < len(self.bot_routes[i]):
                        cur_obj = self.bot_routes[i][step]
                    else:
                        cur_obj = None
                if cur_obj != None:
                    temp_instructions = self.check_turning_or_rotation(ori,cur_obj)
                    
                    #Rotation
                    if temp_instructions[0] == 0:
                        instruction = 4
                        ammount = blocks
                        adv = copy.deepcopy(self.get_ori_vector(ori))
                        for b in range(0,blocks):
                            pos[0] += adv[0]
                            pos[1] += adv[1]
                        self.bot_instructions[i].append([instruction,ammount])
                        #print("INSTRUCTION ADDED ({},{}):\n{} added forward advance of {} blocks".format(instruction, ammount, bot[0],blocks))
                
                    #Turning
                    else:
                        vector = []
                        if temp_instructions[1] == 0:
                            #Turn right after some blank blocks
                            vector = self.get_ori_vector(self.turn_ori(ori,3))
                        else:
                            #Turn left after some blank blocks
                            vector = self.get_ori_vector(self.turn_ori(ori,1))
                        inBlock = False
                        empty_blocks = 0
                        for b in range(0,blocks+1):
                            adj_pos = [pos[0] + vector[0], pos[1] + vector[1]]
                            
                            if inBlock:
                                #If the adjacent block is solid

                                if self.map[adj_pos[1]][adj_pos[0]] != 1:
                                    #If finds that the adj block is empty
                                    empty_blocks += 1
                                    inBlock = False

                            else:
                                #If the adjacent block is empty
                                
                                if self.map[adj_pos[1]][adj_pos[0]] == 1:
                                    #If finds that the adj block is solid
                                    inBlock = True

                            if b < blocks:
                                adv = self.get_ori_vector(ori)
                                pos[0] += adv[0]
                                pos[1] += adv[1]
                        instruction = 1 + temp_instructions[1]
                        ammount = empty_blocks-1
                        ori = cur_obj
                        self.bot_instructions[i].append([instruction,ammount])
                        #print("INSTRUCTION ADDED ({},{}):\n{} added turning in direction {} after an advand of {} empty blocks".format(instruction, ammount, bot[0],instruction,empty_blocks))

                else:
                    self.bot_instructions[i].append([4,blocks])
                    #print("INSTRUCTION ADDED ({},{}):\n{} added forward advance of {} blocks".format(4, blocks, bot[0],blocks))
            print("{}'s list of instructions is: {}".format(bot[0],self.bot_instructions[i]))



    def set_up_task(self):
        #Get map
        self.map_selection()
        #Get robot configuration
        self.robot_configuration_selection()
        #Get reward configuration
        self.reward_configuration_selection()
        #Set each bot Target
        self.set_bots_target()
        #Set each bot path
        self.set_bots_path()
        #Translate the path to a route
        self.set_bots_route()
        #Set the instructions for the robots
        self.set_bot_instructions()
        print("_______________________________")
        print("_______________________________")
        print("TASK ALL SET UP")
        print("_______________________________")
        print("_______________________________")

    def initialize_ros(self):
        pass

    def __init__(self):
        self.route = "./../data/"
        self.map = None
        self.bot_map = None
        self.rwd_map = None
        self.map_name = ""
        self.bot_configuration = None
        self.rwd_configuration = None
        self.targeted_rewards = None
        self.width = 0
        self.height = 0
        self.size = 0
        self.neg_map = None
        self.bot_paths = None
        self.bot_routes = None
        self.bot_instructions = None
        self.bot_rotation_matrix = [[0,2,3,1],[2,0,1,3],[1,3,0,2],[3,1,2,0]]
        print("The pick rewards task consists of giving instructions to a group of robots to find and 'pick-up' some rewards on the environment.")

        #Set upo the task
        self.set_up_task()

        #Initialize ROS


