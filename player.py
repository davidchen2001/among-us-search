from environment import Node, PriorityQueue

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
        self.queue = PriorityQueue()
        self.explored = []
        self.path = []

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
    
    def getNode(self):
        return Agent.getNode(self)

    def get_alive(self):
        return self.alive
    
    def set_alive(self, val):
        self.alive = val
    
    def get_queue(self):
        return self.queue
    
    def append_to_queue(self, node, priority=0):
        self.queue.add(node, priority)
    
    def pop_from_queue(self):
        return self.queue.pop()
    
    def get_explored(self):
        return self.explored
    
    def append_to_explored(self, node):
        self.explored.append(node)
    
    def get_path(self):
        return self.path
    
    def append_to_path(self, node):
        self.path.append(node)



class Killer(Agent):
    
    def __init__(self, node):
        Agent.__init__(self, node)

    def heuristic(self, node, crewmates_locations, remaining_crewmates):
        surviving_crewmates = list.copy(crewmates_locations)
        print(len(surviving_crewmates))

        distances = []

        for crewmate_location in surviving_crewmates:
            print(crewmate_location)
            distances.append(abs(crewmate_location[0] - node[0]) + abs(crewmate_location[1] - node[1]))
    
        #Goal state achieved if no crewmates remain
        if len(distances) == 0:
            return 0
        
        #return h(n) = #of crewmates remaining + mininimum manhattan
        return remaining_crewmates + min(distances)
    
    def getNode(self):
        return Agent.getNode(self)



        
