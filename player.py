from environment import Node

class Agent:
    def __init__(self, node):
        self.currNode = node
    
    def getNode(self):
        return self.currNode

    def setNode(self, node):
        self.currNode = node

class Crewmate(Agent):
    def __init__(self, node):
        Agent.__init__(self, node)
        self.alive = True 

    def heuristic(self, node, tasks):
        new_tasks = list.copy(tasks)

        if self.currNode in new_tasks:
            new_tasks.remove(node)

        distances = []

        for task in new_tasks:
            distances.append(abs(task[0]-node[0]) + abs(task[1]-node[1]))
    
        #Goal state achieved if no tasks exists
        if len(distances) == 0:
            return 0
        
        #return h(n) = #of tasks remaining + mininimum manhattan
        return len(new_tasks) + min(distances)
    
    def get_alive(self):
        return self.alive
    
    def set_alive(self, val):
        self.alive = val

class Killer(Agent):
    
    def __init__(self, node):
        Agent.__init__(self, node)

    def heuristic(self, node, crewmates, remaining_crewmates):
        surviving_crewmates = list.copy(crewmates)

        #TODO: Implement Kill Function

        #if self.currNode.getX() == :
        #    new_tasks.remove(self.currNode)

        distances = []

        for crewmate in surviving_crewmates:
            distances.append(abs(crewmate.getNode()[0]-node[0]) + abs(crewmate.getNode()[1]-node[1]))
    
        #Goal state achieved if no crewmates remain
        if len(distances) == 0:
            return 0
        
        #return h(n) = #of crewmates remaining + mininimum manhattan
        return remaining_crewmates + min(distances)



        
