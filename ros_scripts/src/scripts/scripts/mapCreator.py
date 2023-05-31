#!/usr/bin/env python3.8
import random
import os
import math
import time
import json

#Class that handles the maps
class MapCreator:
    #Returns all of the edges that connect the tree to vertices that are not a part of it. It has a 'DEL' probability to add an edge that will create a cycle.
    def getAdjEdges(self, A,L, cycle,w,h):
        R = []
        for i in range(0,len(A)):
            y, x = A[i][1], A[i][0]
            #print("x is {} and y is {}".format(x,y))
            # UP
            if y > 0:
            #print("y>0")
                if L[y-1][x] > 0:
                    if [x,y-2] not in A:
                        R.append([[x,y-1],L[y-1][x],[0,-1]])
                    else:
                        if random.randint(1,1000) < cycle:
                            R.append([[x,y-1],L[y-1][x],[0,-1]])  
            # DOWN
            if y < (2*h-2):
            #print("y<h-2")
                if L[y+1][x] > 0:
                    if [x,y+2] not in A:
                        R.append([[x,y+1],L[y+1][x],[0,1]])
                else:
                    if random.randint(1,1000) < cycle:
                        R.append([[x,y+1],L[y+1][x],[0,1]])  
            # LEFT
            if x > 0:
            #print("x>0")
                if L[y][x-1] > 0:
                    if [x-2,y] not in A:
                        R.append([[x-1,y],L[y][x-1],[-1,0]])
                else:
                    if random.randint(1,1000) < cycle:
                        R.append([[x-1,y],L[y][x-1],[-1,0]])  
            # RIGHT
            if x < (2*w-2):
            #print("x<w-2")
                if L[y][x+1]:
                    if [x+2,y] not in A:
                        R.append([[x+1,y],L[y][x+1],[1,0]])
                else:
                    if random.randint(1,1000) < cycle:
                        R.append([[x+1,y],L[y][x+1],[1,0]])  
        return R

    ## Creating the maze array. 0 for path, > 0 for path
    def createMap(self, w, h, cycle, wall):
        ### Creating a matrix to store each tile. The matrix will have 2*w-1 columns and 2*h-1 rows. Initializes each tile as '-1'
        L = [[-1 for _ in range(2*w-1)] for _ in range(2*h-1)]


        for i in range(0,len(L)):
            for j in range(0,len(L[i])):
                #For each cell:

                #If both indexes are even, mark it with a 'zero', so it's a 'vertex' in the graph
                if (i%2 == 0) and (j%2 == 0):
                    L[i][j] = 0

                #If both indexes are odd, mark it with a 'one', so it's an automatic 'wall' since these cells are not edges that connect two vertices.
                elif (i%2 != 0) and (j%2 != 0):
                    if random.randint(0,1000) < wall:
                        L[i][j] = 1
                    else:
                        L[i][j] = 0
                
                #For any other case (the edges that connect the vertices) put a random weight between 1 and the ammount of cells in the matrix, so that a MST algorithm can be implemented to create the maze
                else:
                    L[i][j] = random.randint(1,(2*h-1)*(2*w-1))


        ## Set the start coordinates for the MST. Since Prim's algorithm will be implemented, a vertex is picked to begin the construction of the tree.
        x, y = random.randint(0,2*w-1),random.randint(0,2*h-1)
        while (x%2 != 0):
            x = random.randint(0,2*w-1) 
        while (y%2 != 0):
            y = random.randint(0,2*h-1)

        #Prints the coordinates of the vertex selected.
        print("x is {} and y is {}".format(x,y))



        #Array with all the vertices on the tree
        T = []
        T.append([x,y]) #Adding the first vertex
        #print(T)
        while len(T) < w*h and cycle < 1000:
            Adj = self.getAdjEdges(T,L,cycle,w,h)  #gets all edges that go to new vertices (plus a probability to add vertices that make cycles)
            #print("Adj: {}".format(Adj))
            min = Adj[0][1]
            minInd = 0

            #Get the edge of min weight
            for i in range(0,len(Adj)):
                if min > Adj[i][1]:
                    min = Adj[i][1]
                    minInd = i
            #x_i and y_i get the coordinate of the min weight edge
            x_i,y_i = Adj[minInd][0][0], Adj[minInd][0][1]
            #print("bef x is {} and y is {}".format(x_i,y_i))
            #print("L before: {}".format(L[y_i][x_i]))
            
            #Set that edge to 'zero' so its no longer a 'wall'
            L[y_i][x_i] = 0
            #print("L after: {}".format(L[y_i][x_i]))

            #Now x_i and y_i will be the coordinates of the vertex that connects the vertex most recently added (or that generates a cycle)
            x_i += Adj[minInd][2][0]
            y_i += Adj[minInd][2][1]
            #print("aft x is {} and y is {}".format(x_i,y_i))

            #Checks if that vertex is already in the MST. If not, it gets added
            if [x_i,y_i] not in T:
                T.append([x_i,y_i])

        #Sets all unselected edges with weight 1
        for i in range(0,len(L)):
            for j in range(0,len(L[i])):
                if L[i][j] > 0:
                    L[i][j] = 1

        return L
    #Init method
    def __init__(self):
        self.route = "./../data/"

    #Checks if the name file_name is an appropriate name for a file. No spaces
    def valid_file_name(self,file_name):
        return all(c.isalnum() or c in ['_', '-'] for c in file_name)

    #Saves the map im the mapList and creates its own folder
    def persist_map(self,mappa):
        list_of_maps = []
        with open(self.route + "mapList.txt",'r') as file:
            maps = file.readlines()
        print("Current maps:")
        for line in maps:
            l = line.strip()
            list_of_maps.append(l)
            print("- {}".format(l))
        print("Input the name of the map: ")
        stepOk = False
        while not stepOk:
            mapName = input()
            if self.valid_file_name(mapName):
                if mapName not in list_of_maps:
                    stepOk = True
                else:
                    print("Please enter an unused name")
            else:
                print("Please enter a valid file name")
        list_of_maps.append(mapName)
        with open(self.route + "mapList.txt", 'w') as file:
            for name in list_of_maps:
                file.write(name + '\n')
        data = {
            "map": mappa,
            "bot_configs": [],
            "rwd_configs": []
        }
        json_data = json.dumps(data)
        with open(self.route + mapName + ".json",'w') as file:
            file.write(json_data)




    #Asks the user for the map parameters and calls the appropriate method for creating it 
    def create_new_map(self):
 
        stepOk = False
        print("Input the width of the map: ")
        while not stepOk:
            try:
                width = int(input())
                if width > 0:
                    stepOk = True
                else:
                    print("Please input a width of at least 1")
            except:
                print("Please input a valid value for the width (an integer greater than 0)")
        stepOk = False
        print("Input the height of the map: ")
        while not stepOk:
            try:
                height = int(input())
                if height > 0:
                    stepOk = True
                else:
                    print("Please input a height of at least 1")
            except:
                print("Please input a valid value for the height (an integer greater than 0)")
        stepOk = False
        print("Input the odds (out of 1000) of creating adding cycles to the MST path: ")
        while not stepOk:
            try:
                prob_cycle = int(input())
                stepOk = True
            except:
                print("Please input a valid value for the odds")
        stepOk = False
        print("Input the odds (out of 1000) of creating adding cycles to the MST path: ")
        while not stepOk:
            try:
                prob_wall = int(input())
                stepOk = True
            except:
                print("Please input a valid value for the odds")  
        print("Creating the map:\nWidth,Height: {},{}\nCycles: {}\nWalls: {}".format(width,height,prob_cycle,prob_wall))
        createdMap = self.createMap(width, height, prob_cycle, prob_wall)
        for i in range(0,len(createdMap)):
            print(createdMap[i])
        self.persist_map(createdMap)


    def edit_previous_map(self):
        print("Select the map to edit: ")
        list_of_maps = []
        with open(self.route + "mapList.txt",'r') as file:
            maps = file.readlines()
        for line in maps:
            l = line.strip()
            list_of_maps.append(l)
        stepOk = False
        if len(list_of_maps) > 0:
            while not stepOk:
                for i in range(0,len(list_of_maps)):
                    print("{} - {}".format(i+1,list_of_maps[i]))
                try:
                    map_index = int(input())
                    map_selected = list_of_maps[map_index-1]
                    stepOk = True
                except:
                    print("Please select a valid map")
            print("You selected the map {}:".format(map_selected))
            with open(self.route + map_selected+".json", 'r') as file:

                data = json.load(file)
            mapp = data['map']
            for i in range(0,len(mapp)):
                print(mapp[i])
            editDone = False
            while not editDone:
                print("Select the action to make:\n1 - Add a Robot Configuration\n2 - View Robot Configurations\n3 - Add a Reward Configuration\n4 - View Reward Configurations\n5 - Exit")
                op = input()
                if op == '1':
                    print("Select the method:\n1 - Random add\n2 - Manual add")
                    op = input()
                    if op == '1':
                        print("Random add selected. Input the number of robots")
                        stepOk = False
                        while not stepOk:
                            try:
                                n = int(input())
                                if n > 0:
                                    stepOk = True
                                else:
                                    print("Please enter a valid number")
                            except:
                                print("Please enter a valid number")
                        current_configs = data["bot_configs"]
                        bot_config = []
                        i, j = 0, 0
                        mappa = []
                        for i in range(0,len(mapp)):
                            mappa.append([])
                            for j in range(0,len(mapp[i])):
                                mappa[i].append(mapp[i][j])
                        w = len(mappa)
                        h = len(mappa[0])
                        proba = w*h
                        while len(bot_config) < n:
                            if j >= len(mappa[i]):
                                j = 0
                                i += 1
                            if i >= len(mappa):
                                i = 0
                            if mappa[i][j] == 0:
                                if random.randint(0,proba) <= n:
                                    mappa[i][j] = 3
                                    BO = math.pi*random.randint(0,3)/2
                                    ori = 'S'
                                    if BO < 1:
                                        ori = 'E'
                                    elif BO < 2:
                                        ori = 'N'
                                    elif BO < 4:
                                        ori  = 'W'
                                    bot_config.append(['epuck'+str(len(bot_config)+1),[j,i],ori])
                            j += 1
                        current_configs.append(bot_config)
                        data["bot_configs"] = current_configs
                        data["map"] = mapp
                        json_data = json.dumps(data)
                        with open(self.route + map_selected + ".json",'w') as file:
                            file.write(json_data)

                    elif op == '2':
                        print("Manual adding selected. Please select the number of robots that you want to add.")
                        stepOk = False
                        while not stepOk:
                            try:
                                n = int(input())
                                if n > 0:
                                    stepOk = True
                                else:
                                    print("Please enter a valid number of robots")
                            except:
                                print("Please enter a valid number of robots")
                        current_configs = data["bot_configs"]
                        bot_config = []
                        mappa = []
                        for i in range(0,len(mapp)):
                            mappa.append([])
                            for j in range(0,len(mapp[i])):
                                mappa[i].append(mapp[i][j])
                        h = len(mappa)
                        w = len(mappa[0])
                        while len(bot_config) < n:
                            print("Adding the robot {}.".format(len(bot_config)+1))
                            stepOk = False
                            while not stepOk:
                                try:
                                    print("Input the column")
                                    x = int(input())
                                    if x >= 0 and x < w:
                                        print("Input the row:")
                                        y = int(input())
                                        if y >= 0 and y < h:
                                            print("Currently at {},{}: {}".format(x,y,mappa[y][x]))
                                            if mappa[y][x] == 0:
                                                print("Input the orientation (N,S,E,W)")
                                                ori = input()
                                                if ori in ['N','S','W','E']:
                                                    mappa[y][x] = 3
                                                    bot_config.append(['epuck'+str(len(bot_config)+1),[x,y],ori])
                                                    stepOk = True
                                                else:
                                                    print("please enter a valid orientation")
                                            else:
                                                print("You can't put a robot here")
                                        else:
                                            print("Please enter a valid row")
                                    else:
                                        print("please enter a valid column")
                                except:
                                    print("Please input a valid coordinate")
                        current_configs.append(bot_config)
                        data["bot_configs"] = current_configs
                        data["map"] = mapp
                        json_data = json.dumps(data)
                        with open(self.route + map_selected + ".json",'w') as file:
                            file.write(json_data)
                elif op == '2':
                    print("These are the current robot configurations for the map:")
                    configs = data["bot_configs"]
                    for i in range(0,len(configs)):
                        print("- {}".format(configs[i]))
                elif op == '3':
                    print("Select the method:\n1 - Random add\n2 - Manual add")
                    op = input()
                    if op == '1':
                        print("Random add selected. Input the number of rewards")
                        stepOk = False
                        while not stepOk:
                            try:
                                n = int(input())
                                if n > 0:
                                    stepOk = True
                                else:
                                    print("Please enter a valid number")
                            except:
                                print("Please enter a valid number")
                        current_configs = data["rwd_configs"]
                        rwd_config = []
                        i, j = 0, 0
                        mappa = []
                        for i in range(0,len(mapp)):
                            mappa.append([])
                            for j in range(0,len(mapp[i])):
                                mappa[i].append(mapp[i][j])
                        w = len(mappa)
                        h = len(mappa[0])
                        proba = w*h
                        while len(rwd_config) < n:
                            if j >= len(mappa[i]):
                                j = 0
                                i += 1
                            if i >= len(mappa):
                                i = 0
                            if mappa[i][j] == 0:
                                if random.randint(0,proba) <= n:
                                    mappa[i][j] = 2
                                    rwd_config.append([j,i])
                            j += 1
                        current_configs.append(rwd_config)
                        data["rwd_configs"] = current_configs
                        data["map"] = mapp
                        json_data = json.dumps(data)
                        with open(self.route + map_selected + ".json",'w') as file:
                            file.write(json_data)

                    elif op == '2':
                        print("Manual adding selected. Please select the number of rewards that you want to add.")
                        stepOk = False
                        while not stepOk:
                            try:
                                n = int(input())
                                if n > 0:
                                    stepOk = True
                                else:
                                    print("Please enter a valid number of rewards")
                            except:
                                print("Please enter a valid number of rewards")
                        current_configs = data["rwd_configs"]
                        rwd_config = []
                        mappa = []
                        for i in range(0,len(mapp)):
                            mappa.append([])
                            for j in range(0,len(mapp[i])):
                                mappa[i].append(mapp[i][j])
                        h = len(mappa)
                        w = len(mappa[0])
                        while len(rwd_config) < n:
                            print("Adding the reward {}.".format(len(rwd_config)+1))
                            stepOk = False
                            while not stepOk:
                                try:
                                    print("Input the column")
                                    x = int(input())
                                    if x >= 0 and x < w:
                                        print("Input the row:")
                                        y = int(input())
                                        if y >= 0 and y < h:
                                            print("Currently at {},{}: {}".format(x,y,mappa[y][x]))
                                            if mappa[y][x] == 0:
                                                mappa[y][x] = 3
                                                rwd_config.append([x,y])
                                                stepOk = True
                                            else:
                                                print("You can't put a reward here")
                                        else:
                                            print("Please enter a valid row")
                                    else:
                                        print("please enter a valid column")
                                except:
                                    print("Please input a valid coordinate")
                        current_configs.append(rwd_config)
                        data["rwd_configs"] = current_configs
                        data["map"] = mapp
                        json_data = json.dumps(data)
                        with open(self.route + map_selected + ".json",'w') as file:
                            file.write(json_data)
                elif op == '4':
                    print("These are the current reward configurations for the map:")
                    configs = data["rwd_configs"]
                    for i in range(0,len(configs)):
                        print("- {}".format(configs[i]))
                elif op == '5':
                    editDone = True
                else:
                    print("Please select a valid option")