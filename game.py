from environment import Graph, Node, PriorityQueue
from player import Crewmate, Killer

import numpy as np
import random

def initialize_crewmates(data, grid, num_crewmates):
    n,d = data.shape
    
    crewmates = []

    for i in range(num_crewmates):
       
        start_node = grid.get_node((1,1))
        task_locations = grid.get_task_cords()

        crewmates.append(Crewmate(start_node))
        crewmates[i].append_to_queue([start_node.get_loc(), list.copy(task_locations), 0, [], None])
        crewmates[i].setNode(start_node.get_loc())
    
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

#Get the path to the node specified from the origin (0,0)
#currNode is the nodes path being searched
#paths is an array of all explored (location, state, parent)
#cost is the cost to get to currNode
def getPath(currNode, paths, cost):
    final_path = [currNode[0]]
    countdown = cost
    parent = currNode[3]
    #Loops through nodes in path, searching for the parent nodes parent to create a path
    while countdown > 0:
        for x in paths:
            if x[0] == parent[0] and x[1] == parent[1] and x[2] == countdown-1:
                final_path.append(x[0])
                parent = x[3]
                countdown -=1
                break
    
    final_path.reverse()
    #return array representing path to get to currNode
    return final_path

def killer_traversal(killer_queue, grid, remaining_crewmates, killer_explored, killer_path, killer, function):

    if len(killer_queue) == 0:
        return False
                
    currNode = killer_queue.pop()
    killer.setNode(currNode[0])
    node_information = grid.get_node(str(currNode[0]))

    #Check if killer collided with crewmate
    num_collisions = 0
    for j in range(len(grid.get_crewmate_locations())):
        if grid.get_crewmate_locations()[j] == currNode[0]:
                num_collisions += 1
                crewmate_at_location = grid.get_crewmate_at_location(grid.get_crewmate_locations()[j])
                crewmate_at_location.set_alive(False)
                grid.append_crewmates_death(grid.get_crewmate_locations()[j])
                grid.remove_agent(crewmate_at_location)
                remaining_crewmates -= 1
                break 
        
        if remaining_crewmates == 0:
            optimal_path_cost = currNode[2]
            killer_optimal_path = getPath(currNode, killer_path, optimal_path_cost)
            return killer_optimal_path, optimal_path_cost, "killer wins"

        killer_explored.append([currNode[0], list.copy(grid.get_crewmate_locations())])
        killer_path.append(currNode)

        curr_path_cost = 1 + currNode[2]

        for node in node_information.neighbours:
            neighbour_node = grid.get_node(node)

            #Create neighbour for adding to frontier
            neighbour = [neighbour_node.get_loc(), list.copy(grid.get_crewmate_locations()), curr_path_cost, [currNode[0], list.copy(grid.get_crewmate_locations())]]

            #Check if neighbour with treasure state already exists in paths to get old_path_cost for next if statement
            old_path_cost = 0
            for x in killer_path:
                if x[0] == neighbour_node.get_loc() and x[1] == grid.get_crewmate_locations():
                    old_path_cost = x[2]

            #Check if neighbour with treasure state is not in explored or if the current path is smaller than the previous neighbour with same treausre states path
            #If if statement passes, add to frontier
            if (([neighbour[0], grid.get_crewmate_locations()] not in killer_explored) 
                or (curr_path_cost < old_path_cost)):
                
                #Calculate f(n) of current location+treasure_state
                        
                if function == "fn":
                    fn = curr_path_cost + killer.heuristic(neighbour[0], grid.get_crewmate_locations(), remaining_crewmates)
                elif function == "hn":
                    fn = killer.heuristic(neighbour[0], grid.get_crewmate_locations(), remaining_crewmates)
                neighbour.append(fn)

                #Add to priority queue, with fn defining it's priority
                killer_queue.add(
                    neighbour,
                    fn
                )

def pathfinding(input, num_crewmates, function):

    data = load_data(input)

    grid = Graph(data)

    remaining_crewmates = num_crewmates
    crewmates = initialize_crewmates(data, grid, remaining_crewmates)
    
    grid.set_agents(crewmates)

    crewmate_locations = grid.get_crewmate_locations()

    killer_queue = PriorityQueue()
    killer_explored = []
    killer_path = []
    start_node = grid.get_node((0,0))
    killer = initialize_killer(start_node)

    killer_queue.add([start_node.get_loc(), list.copy(crewmate_locations), 0, [], None])

    while True:
        crewmates = grid.get_agents()

        if len(crewmates) == 0:
            currNode = killer_queue.pop()
            optimal_path_cost = currNode[2]
            killer_optimal_path = getPath(currNode, killer_path, optimal_path_cost)
            return killer_optimal_path, optimal_path_cost, "killer wins", grid.get_agents_death()

        for crewmate in crewmates:

            crewmate_queue = crewmate.get_queue()
        
            if len(crewmate_queue) == 0:
                return False 
                
            currNode = crewmate.pop_from_queue()
            crewmate.setNode(currNode[0])
            node_information = grid.get_node(str(currNode[0]))

            #Remove task if agent has completed it
            if (currNode[0] in grid.get_task_cords()):
                grid.remove_task_cords(currNode[0])
                
            if len(grid.get_task_cords()) == 0:
                return "crewmate wins", grid.get_agents_death()

            crewmate.append_to_explored([currNode[0], list.copy(grid.get_task_cords())])
            currNode[1] = list.copy(grid.get_task_cords())
            crewmate.append_to_path(currNode)

            curr_path_cost = 1 + currNode[2]

            for node in node_information.neighbours:
                neighbour_node = grid.get_node(node)

                #Create neighbour for adding to frontier
                neighbour = [neighbour_node.get_loc(), list.copy(grid.get_task_cords()), curr_path_cost, [currNode[0], list.copy(grid.get_task_cords())]]

                #Check if neighbour with treasure state already exists in paths to get old_path_cost for next if statement
                old_path_cost = 0
                for x in crewmate.get_path():
                    if x[0] == neighbour_node.get_loc() and x[1] == grid.get_task_cords():
                        old_path_cost = x[2]

                #Check if neighbour with treasure state is not in explored or if the current path is smaller than the previous neighbour with same treausre states path
                #If if statement passes, add to frontier
                if (([neighbour[0], grid.get_task_cords()] not in crewmate.get_explored()) 
                or (curr_path_cost < old_path_cost)):
                    #Calculate f(n) of current location+treasure_state

                    if function == "fn":
                        fn = curr_path_cost + crewmate.heuristic(neighbour[0], grid.get_task_cords())
                    elif function == "hn":
                        fn = crewmate.heuristic(neighbour[0], grid.get_crewmate_locations())

                    neighbour.append(fn)

                    #Add to priority queue, with fn defining it's priority
                    crewmate.append_to_queue(
                        neighbour,
                        fn
                    )
            
            killer_traversal(killer_queue, grid, remaining_crewmates, killer_explored, killer_path,killer, function)
