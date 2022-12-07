from environment import Graph, Node, PriorityQueue
from player import Crewmate, Killer

import numpy as np

def initialize_crewmates(node):
    crewmates = []

    for i in range(4):
        crewmates.append(Crewmate(node))
    
    return crewmates

def initialize_killer(node):

    killer = Killer(node)
    return killer

#load_data from a csv_file into a numpy matrix
#csv_file represents 
def load_data(csv_file):
    data = np.genfromtxt(csv_file, delimiter=',', dtype=None, encoding=None)
    #data represents numpy matrix being returned
    return data

def pathfinding(input):

    data = load_data(input)
    grid = Graph(data)
    task_locations = grid.get_task_cords()

    crewmates_queues = []
    crewmates_explored = []
    crewmates_path = []
    remaining_crewmates = 4
    start_node = grid.get_node((0,0))
    crewmates = initialize_crewmates(start_node)
    killer = initialize_killer(start_node)

    grid.set_agents(crewmates)

    crewmate_locations = grid.get_crewmate_locations()

    for i in range(remaining_crewmates):

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
    killer_queue.add([start_node.get_loc(), list.copy(crewmate_locations), 0, [], None])

    while True:
        crewmates = grid.get_agents()

        for i in range(remaining_crewmates+1):

            if i == remaining_crewmates: 
                if len(killer_queue) == 0:
                    return False
                
                currNode = killer_queue.pop()
                #agent_state = list.copy(currNode[1])
                node_information = grid.get_node(str(currNode[0]))

                #Check if killer collided with crewmate
                num_collisions = 0
                for j in range(len(grid.get_crewmate_locations())):
                    if grid.get_crewmate_locations()[j] == currNode[0]:
                        num_collisions += 1
                        crewmate_at_location = grid.get_crewmate_at_location(grid.get_crewmate_locations()[j])
                        crewmate_at_location.set_alive(False)
                        grid.remove_agent(crewmate_at_location)
                        remaining_crewmates -= 1
                        break 

                if remaining_crewmates == 0:
                    optimal_path_cost = currNode[2]
                    return killer_path, optimal_path_cost, "killer"

                killer_explored.append([currNode[0], list.copy(grid.get_crewmate_locations())])
                #currNode[1] = list.copy(agent_state)
                killer_path.append(currNode)

                curr_path_cost = 1 + currNode[2]

                for node in node_information.neighbours:
                    neighbour_node = grid.get_node(node)

                    #Create neighbour for adding to frontier
                    neighbour = [neighbour_node.get_loc(), list.copy(grid.get_crewmate_locations()), curr_path_cost, [currNode[0], list.copy(grid.get_crewmate_locations())]]

                    #Check if neighbour with treasure state already exists in paths to get old_path_cost for next if statement
                    old_path_cost = 0
                    for x in paths:
                        if x[0] == neighbour_node.get_loc() and x[1] == grid.get_crewmate_locations():
                            old_path_cost = x[2]

                    #Check if neighbour with treasure state is not in explored or if the current path is smaller than the previous neighbour with same treausre states path
                    #If if statement passes, add to frontier
                    if (([neighbour[0], grid.get_crewmate_locations()] not in explored) 
                    or (curr_path_cost < old_path_cost)):
                        #Calculate f(n) of current location+treasure_state
                        
                        fn = curr_path_cost + killer.heuristic(neighbour[0], grid.get_crewmate_locations(), remaining_crewmates)
                        neighbour.append(fn)

                        #Add to priority queue, with fn defining it's priority
                        killer_queue.add(
                            neighbour,
                            fn
                        )

            else:
                if len(crewmates_queues[i]) == 0:
                    return False 
                
                currNode = crewmates_queues[i].pop()
                node_information = grid.get_node(str(currNode[0]))

                if (currNode[0] in grid.get_task_cords()):
                    grid.remove_task_cords(currNode[0])
                
                if len(grid.get_task_cords()) == 0:
                    return "crewmate"

                crewmates_explored[i].append([currNode[0], list.copy(grid.get_task_cords())])
                currNode[1] = list.copy(grid.get_task_cords())
                crewmates_path[i].append(currNode)

                curr_path_cost = 1 + currNode[2]

                for node in node_information.neighbours:
                    neighbour_node = grid.get_node(node)

                    #Create neighbour for adding to frontier
                    neighbour = [neighbour_node.get_loc(), list.copy(grid.get_task_cords()), curr_path_cost, [currNode[0], list.copy(grid.get_task_cords())]]

                    #Check if neighbour with treasure state already exists in paths to get old_path_cost for next if statement
                    old_path_cost = 0
                    for x in paths:
                        if x[0] == neighbour_node.get_loc() and x[1] == grid.get_task_cords():
                            old_path_cost = x[2]

                    #Check if neighbour with treasure state is not in explored or if the current path is smaller than the previous neighbour with same treausre states path
                    #If if statement passes, add to frontier
                    if (([neighbour[0], grid.get_task_cords()] not in explored) 
                    or (curr_path_cost < old_path_cost)):
                        #Calculate f(n) of current location+treasure_state
                        fn = curr_path_cost + crewmates[i].heuristic(neighbour[0], grid.get_task_cords())
                        neighbour.append(fn)

                        #Add to priority queue, with fn defining it's priority
                        crewmates_queues[i].add(
                            neighbour,
                            fn
                        )


print(pathfinding("./Test.csv"))