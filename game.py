from environment import Graph, Node, PriorityQueue
from player import Crewmate, Killer

def initialize_agents():
    crewmates = []

    for i in range(5):
        crewmates.append(Crewmate())
    
    killer = Killer()
    return crewmates, killer


def pathfinding(input):
    crewmates, killer = initialize_agents()

    crewmate_queue = PriorityQueue()
    killer_queue = PriorityQueue()

    



    