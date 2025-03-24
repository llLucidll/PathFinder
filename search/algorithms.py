import heapq
import math

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def set_parent(self, parent):
        """
        Sets the parent of a node in the search tree
        """
        self._parent = parent

    def get_parent(self):
        """
        Returns the parent of a node in the search tree
        """
        return self._parent
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost

import heapq
import math

class Dijkstra:
    def __init__(self, gridded_map):
        self._gridded_map = gridded_map
        # self.open = []
        # self.closed = {}
        # self.expand = 0
        self.path = []
    
    def search(self, start, goal):
        
        self.open = []
        self.closed = {}
        self.expand = 0
        
        # Initialize best_costs with the start state.
        best_costs = {start.state_hash(): 0}
        start.set_cost(0)  
        heapq.heappush(self.open, start)

        while self.open:
            current = heapq.heappop(self.open)
            
            # Skip this node if a better path was already found.
            if current.get_cost() > best_costs.get(current.state_hash(), float('inf')):
                continue

            self.closed[current.state_hash()] = current

            # Check if we've reached the goal.
            if current.state_hash() == goal.state_hash():
                return self.build_path(goal), current.get_cost(), self.expand

            # Expand current's neighbors.
            successors = self._gridded_map.successors(current)
            for successor in successors:
                
                step_cost = self._gridded_map.cost(
                    successor.get_x() - current.get_x(),
                    successor.get_y() - current.get_y()
                )
                new_cost = current.get_cost() + step_cost

                # If this successor hasn't been seen or we found a lower cost.
                if new_cost < best_costs.get(successor.state_hash(), float('inf')):
                    best_costs[successor.state_hash()] = new_cost
                    successor.set_parent(current)
                    successor.set_cost(new_cost)
                    heapq.heappush(self.open, successor)

            self.expand += 1
        
        
        return None, -1, self.expand

    def build_path(self, goal):
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = current.get_parent()
        path = path.reverse() 
        return path

    def get_closed_data(self):
        return self.closed


import heapq
import math

class AStar:
    def __init__(self, gridded_map):
        self._gridded_map = gridded_map
        self.path = []
        self.expand = 0

    def heuristic(self, dx, dy):
        """
        Heuristic function: estimates the cost from a given state to the goal.
        Uses the formula: 1.5 * min(dx, dy) + abs(dx - dy)
        where dx and dy are the absolute differences in the x and y coordinates.
        """
        return 1.5 * min(dx, dy) + abs(dx - dy)

    def search(self, start, goal):
        self.open = []
        self.closed = {}
        self.expand = 0
        
        
        best_costs = {start.state_hash(): 0}
        
        
        start.set_g(0)
        h = self.heuristic(abs(start.get_x() - goal.get_x()),
                           abs(start.get_y() - goal.get_y()))
        start.set_cost(0 + h)  
        heapq.heappush(self.open, start)

        while self.open:
            
            current = heapq.heappop(self.open)
            
            # Skip this node if a better path was already found.
            if current.get_g() > best_costs.get(current.state_hash(), float('inf')):
                continue

            self.closed[current.state_hash()] = current

            # Check if we've reached the goal.
            if current.state_hash() == goal.state_hash():
                
                return self.build_path(current), current.get_g(), self.expand

            # Expand the current node.
            successors = self._gridded_map.successors(current)
            for successor in successors:
                
                step_cost = self._gridded_map.cost(
                    successor.get_x() - current.get_x(),
                    successor.get_y() - current.get_y()
                )
                new_g = current.get_g() + step_cost

                # Compute the heuristic 
                h = self.heuristic(abs(successor.get_x() - goal.get_x()),
                                   abs(successor.get_y() - goal.get_y()))
                new_f = new_g + h

                # If this path to the successor is better than any previous one...
                if new_g < best_costs.get(successor.state_hash(), float('inf')):
                    best_costs[successor.state_hash()] = new_g
                    successor.set_parent(current)
                    successor.set_g(new_g)
                    successor.set_cost(new_f)  # f = g + h
                    heapq.heappush(self.open, successor)

            self.expand += 1

        # If the loop ends without reaching the goal, return an indicator.
        return None, -1, self.expand

    def build_path(self, goal_node):
        
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = current.get_parent()
        path.reverse()
        return path

    def get_closed_data(self):
        return self.closed
