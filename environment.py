import numpy as np
import heapq

#Priority Queue to store explored locations
class PriorityQueue:
    def __init__(self):
        self.heap = []

    def add(self, val, key=0):
        heapq.heappush(self.heap, (key, val))
    
    def pop(self):
        key, val = heapq.heappop(self.heap)
        return val

    def findNode(self, node):

        for item in self.heap:
            if str(node) == str(item):
                return True
        
        return False

    def __len__(self):
        return len(self.heap)

#Node that stores a grid positions information
class Node:
    def __init__(self, val, cords, neighbours):
        self.val = val                  
        self.x = cords[0]
        self.y = cords[1] 
        self.name = cords               #Stores coordinates of location
        self.neighbours = neighbours    #Stores Neighbours

    def print_node_details(self):
        arr = []
        for node in self.neighbours:
            arr.append(str(node))

    def __str__(self):
        return str(self.name)
        
    def get_neighbours(self):
        return self.neighbours

    def get_loc(self):
        return self.name

    def get_val(self):
        return self.val
    
    def set_val(self, val):
        self.val = val

#Graph that stores initial environments
class Graph:
    def __init__(self,matrix):
        n,d = matrix.shape
        self.nodes = []             #Stores all locations
        self.task_count = 0     #Stores number of tasks in grid
        self.width = n              #Gets width of grid
        self.height = d             #Gets height of matrix
        self.task_cords = []
        self.agents = []

        #Get location information to create Nodes for the Graph, i.e neighbours of location, location value
        for x, y in np.ndindex(matrix.shape):
            neighbours = []
            #Count tasks for task_count
            if matrix[x,y] == 'T':
                self.task_count += 1
                self.task_cords.append((x,y))

            possible_neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
            if x+1>=n:
                possible_neighbors.remove((x+1,y))
            if x-1<0:
                possible_neighbors.remove((x-1,y))
            if y+1>=d:
                possible_neighbors.remove((x,y+1))
            if y-1<0:
                possible_neighbors.remove((x,y-1))
            neighbours = possible_neighbors
            node = Node(matrix[x,y],(x,y),neighbours)
            self.nodes.append(node)

    def get_task_cords(self):
        return self.task_cords
    
    def remove_task_cords(self,cords):
        self.task_cords.remove(cords)

    def print_graph_details(self):
        for node in self.nodes:
            node.print_node_details()

    def get_node(self, other_node):
        for node in self.nodes:
            if str(node) == str(other_node):
                return node

    def print_all(self):
        for node in self.nodes:
            if node.get_val() == "T":
                print("T still exists")
                return
        print("T does not exist")   
    
    def get_agents(self):
        return self.agents

    def get_crewmate_locations(self):
        locations = []

        for i in range(len(self.agents)):
            locations.append(self.agents[i].getNode().get_loc())

        return locations 

    def set_agents(self, agents):
        self.agents = agents 
    
    def append_agent(self, agent):
        self.agents.append(agent)
    
    def remove_agent(self,agent):
        self.agents.remove(agent)