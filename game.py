from environment import Graph, Node, PriorityQueue
from player import Crewmate, Killer

import numpy as np

def initialize_agents():
    crewmates = []

    for i in range(4):
        crewmates.append(Crewmate())
    
    killer = Killer()
    return crewmates, killer

#load_data from a csv_file into a numpy matrix
#csv_file represents 
def load_data(csv_file):
    data = np.genfromtxt(csv_file, delimiter=',', dtype=None, encoding=None)
    #data represents numpy matrix being returned
    return data

def pathfinding(input):
    crewmates, killer = initialize_agents()

    data = load_data(input)
    grid = Graph(data)
    task_locations = grid.get_task_cords()
    agent_locations = grid.get_agent_locations()

    crewmates_queues = []
    crewmates_explored = []
    crewmates_path = []
    start_node = grid.get_node((0,0))

    for i in range(4):

        crewmate_queue = PriorityQueue()
        crewmate_queue.add([start_node.get_loc(), list.copy(task_locations), 0, [], None])
        crewmates_queues.append(crewmate_queue)
        explored = []
        paths = []

        crewmates_explored.append(explored)
        crewmates_path.append(paths)


    killer_queue = PriorityQueue()
    killer_explored = []
    killer_path = []
    start_node = grid.get_node((0,0))
    killer_queue.add([start_node.get_loc(), list.copy(agent_locations), 0, [], None])

    while True:

        for i in range(5):

            if i == 4: 
                if len(killer_queue) == 0:
                    return False
                
                currNode = killer_queue.pop()
                agent_state = list.copy(currNode[1])
                node_information = grid.get_node(str(currNode[0]))

                #Check if killer collided with crewmate
                num_collisions = 0
                for j in range(len(agent_state)):
                    if agent_state[j] == currNode[0] and crewmates[i].get_alive()==True:
                        num_collisions += 1
                
                #num_collisions should be two since killer's location is also collected
                if num_collisions == 2:
                    #Remove agent from the map
                    crewmates[i].set_alive(False)

            else:
                if len(crewmate_queue[i]) == 0:
                    return False 
                



            





    


    