import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """
    MANHATTAN=0
    EUCLIDEAN=1

    # A* currently configured for Manhattan; can be modified for Euclidean
    def __init__(self, parent=None, position=None, method=MANHATTAN):
        self.parent = parent
        self.position = np.array(position)
        self.method = method

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return (self.position == other.position).all()

    # heuristic calculations
    def manhattan_dist(a: np.ndarray, b: np.ndarray):
        return np.sum(np.abs(a-b))
    def euclidean_dist(a: np.ndarray, b: np.ndarray):
        return np.sum((a-b)**2)**.5

    # calculating heuristic cost is as simple as a-b
    def __sub__(self, other):
        if self.method == Node.EUCLIDEAN:
            return Node.euclidean_dist(self.position, other.position)
        elif self.method == Node.MANHATTAN:
            return Node.manhattan_dist(self.position, other.position)
        else:
            raise NotImplementedError("Invalid Method")

    # pretty print for debugging
    def __str__(self):
        return f"node at {self.position} g {self.g} h {self.h} f {self.f}"

#This function return the path of the search
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path

def search(maze, start, end, mazeOrigin):

    print("searching ....")

    maze = maze.T
    
    
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # Nodes are initialized with 0 cost by default
    start_node = Node(position=start)
    end_node = Node(position=end)

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []  
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = [] 
    
    # Add the start node
    yet_to_visit_list.append(start_node)
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # moves are represented as an increment to the current position
    move  =  [[0,1], # go up
              [-1,0], # go left
              [0,-1], # go down
              [1,0], # go right
            #   [-1,1], # go up left
            #   [-1,-1], # go down left
            #   [1,1], # go up right
            #   [1,-1], # go down right
              ]


    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    # unpack the size of the occupancy grid
    no_rows, no_columns = maze.shape
    

    # Loop until you find the end
    
    while len(yet_to_visit_list) > 0:
        
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1    

        
        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        print(current_node)
                
        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:

            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # calculate the neighbour based on the earlier move definition
            node_position = new_position + current_node.position

            # skip the pixels that fall outside the occupancy grid
            if node_position[0] >= no_columns or node_position[1] >= no_rows:
                continue

            # Make sure walkable terrain
            if maze[node_position[0],node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        
        for child in children:
  
            # comparison used by the `in` keyword uses __eq__()
            if child in visited_list:
                continue

            # "real" cost increments by 1 from the parent
            child.g = child.parent.g + 1

            ## Heuristic costs calculated here, this is using eucledian distance
            # the heuristic algo changes based on the default val in the
            # constructor argument for Node
            child.h = end_node - child

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in yet_to_visit_list if child == i and child.g >= i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)
