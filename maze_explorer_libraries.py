import numpy as np

# Size of the maze
maze_size = (250, 400)

class Node:

    # To save the index of the current node
    node_index = 0

    # To save the index of the parent node
    parent_index = 0

    # A list of all the possible actions [North, East, South, West, North-East, South-East, South-West, North-West]
    possible_actions = np.zeros(8, dtype = 'bool')

    # Cost to this node
    cost_to_come = 0

    # To save the center location
    center_location = (0, 0)

    # Count of possible child nodes
    child_nodes = 0

    # Constructor - To be used later
    def __init__(self) -> None:
        pass

    # Method to print all the stored information of the node
    def print_current_values (self) -> None:
        print(f'The index of the current node is {self.node_index}')
        print(f'The index of the parent node {self.parent_index}')
        print(f'The cost to come of the current node is {self.cost_to_come}')
        print(f'The possible actions of the current node are {self.possible_actions}')
        print(f'The location of the center is {self.center_location}')
        print(f'The possible child nodes are {self.child_nodes}')

    # Updating the node
    # Input: The location of the node, index and visited, open and obstacle lists
    # Output: 1. updates the values of the center location, parent location, possible actions and number of child nodes
    #         2. Returns True if there is a possibility to move
    def update_node (self, location, index, list_1, list_2, list_3):
        self.center_location = location
        self.parent_index = index
        self.action_checker(list_1, list_2, list_3)

    # Method to check all the possible directions of the current node
    # Input: The current node and visited, open and obstacle lists
    # Output: Returns True if there is a possibility to move
    def action_checker (self, list_1, list_2, list_3):
        
        self.possible_actions = [self.check_north(self.center_location, list_1, list_2, list_3), 
                                self.check_east(self.center_location, list_1, list_2, list_3), 
                                self.check_south(self.center_location, list_1, list_2, list_3), 
                                self.check_west(self.center_location, list_1, list_2, list_3),
                                self.check_north_east(self.center_location, list_1, list_2, list_3), 
                                self.check_south_east(self.center_location, list_1, list_2, list_3), 
                                self.check_south_west(self.center_location, list_1, list_2, list_3), 
                                self.check_north_west(self.center_location, list_1, list_2, list_3)]
        self.child_count()
        if self.child_nodes != 0:
            return True
        else:
            return False

    # Method to update the number of child nodes
    # Input: The node
    # Output: Updates the child nodes in the node object
    def child_count(self):
        self.child_nodes = np.count_nonzero(self.possible_actions)
    
    # Method to check if the specified node can move North
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move North
    def check_north (self, node, list_1, list_2, list_3):

        if ((node[0] - 1) == -1) or \
            node_finder(((node[0] - 1), node[1]), list_1) or \
            node_finder(((node[0] - 1), node[1]), list_2) or \
            node_finder(((node[0] - 1), node[1]), list_3):
            return False
        else:
            return True

    # Method to check if the specified node can move East
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move East
    def check_east (self, node, list_1, list_2, list_3):

        if ((node[1] + 1) >= maze_size[1]) or \
            node_finder((node[0], (node[1] + 1)), list_1) or \
            node_finder((node[0], (node[1] + 1)), list_2) or \
            node_finder((node[0], (node[1] + 1)), list_3):        
            return False
        else:
            return True

    # Method to check if the specified node can move South
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move South
    def check_south (self, node, list_1, list_2, list_3):
    
        if ((node[0] + 1) >= maze_size[0]) or \
            node_finder(((node[0] + 1), node[1]), list_1) or \
            node_finder(((node[0] + 1), node[1]), list_2) or \
            node_finder(((node[0] + 1), node[1]), list_3):
            return False
        else:
            return True
    
    # Method to check if the specified node can move West
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move West
    def check_west (self, node, list_1, list_2, list_3):
        
        if ((node[1] - 1) == -1) or \
            node_finder((node[0], (node[1] - 1)), list_1) or \
            node_finder((node[0], (node[1] - 1)), list_2) or \
            node_finder((node[0], (node[1] - 1)), list_3):
            return False
        else:
            return True

    # Method to check if the specified node can move North-East
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move North-East
    def check_north_east (self, node, list_1, list_2, list_3):

        if ((node[0] - 1) == -1) or ((node[1] + 1) >= maze_size[1]) or \
            node_finder(((node[0] - 1), (node[1] + 1)), list_1) or \
            node_finder(((node[0] - 1), (node[1] + 1)), list_2) or \
            node_finder(((node[0] - 1), (node[1] + 1)), list_3):
            return False
        else:
            return True

    # Method to check if the specified node can move South-East
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move South-East
    def check_south_east (self, node, list_1, list_2, list_3):

        if ((node[0] + 1) >= maze_size[0]) or ((node[1] + 1) >= maze_size[1]) or \
            node_finder(((node[0] + 1), (node[1] + 1)), list_1) or \
            node_finder(((node[0] + 1), (node[1] + 1)), list_2) or \
            node_finder(((node[0] + 1), (node[1] + 1)), list_3):
            return False
        else:
            return True

    # Method to check if the specified node can move South-West
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move South-West
    def check_south_west (self, node, list_1, list_2, list_3):

        if ((node[0] + 1) >= maze_size[0]) or ((node[1] - 1) == -1) or \
            node_finder(((node[0] + 1), (node[1] - 1)), list_1) or \
            node_finder(((node[0] + 1), (node[1] - 1)), list_2) or \
            node_finder(((node[0] + 1), (node[1] - 1)), list_3):
            return False
        else:
            return True

    # Method to check if the specified node can move North-West
    # Input: Node location and visited, open and obstacle lists
    # Output: True if we can move North-West
    def check_north_west (self, node, list_1, list_2, list_3):

        if ((node[0] - 1) == -1) or ((node[1] - 1) == -1) or \
            node_finder(((node[0] - 1), (node[1] - 1)), list_1) or \
            node_finder(((node[0] - 1), (node[1] - 1)), list_2) or \
            node_finder(((node[0] - 1), (node[1] - 1)), list_3):
            return False
        else:
            return True

    # Method to move North
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_north (self, node, parent_cost, current_cost):
        self.center_location = (node[0] - 1, node[1])
        self.cost_to_come = np.round(parent_cost + current_cost, 2)
        
    # Method to move East
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_east (self, node, parent_cost, current_cost):
        self.center_location = (node[0], node[1] + 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)
        
    # Method to move South
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_south (self, node, parent_cost, current_cost):
        self.center_location = (node[0] + 1, node[1])
        self.cost_to_come = np.round(parent_cost + current_cost, 2)
        
    # Method to move West
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_west (self, node, parent_cost, current_cost):
        self.center_location = (node[0], node[1] - 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)

    # Method to move North-East
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_north_east (self, node, parent_cost, current_cost):
        self.center_location = (node[0] - 1, node[1] + 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)

    # Method to move South-East
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_south_east (self, node, parent_cost, current_cost):
        self.center_location = (node[0] + 1, node[1] + 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)

    # Method to move South-West
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_south_west (self, node, parent_cost, current_cost):
        self.center_location = (node[0] + 1, node[1] - 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)
    
    # Method to move North-West
    # Input: The node, node location, parent cost, cost to reach current node
    # Output: Updates the node location and cost to come
    def move_north_west (self, node, parent_cost, current_cost):
        self.center_location = (node[0] - 1, node[1] - 1)
        self.cost_to_come = np.round(parent_cost + current_cost, 2)

# Iterating through the visited list for checking visited
# Input: location and list of locations
# Output: True if found the node
def found_node (node, visited_stack):
    
    flag = False
    
    for i in visited_stack:
        if node == i:
            flag = True
            break
    return flag 


# Method to check if the node is present in the list/queue
# Input: node's location, list/stack
# Output: True if node is found
def node_finder (node_location, node_list):

    if not node_list:
        return False
    elif found_node(node_location, node_list):
        return True
    else:
        return False

# Method to check if the location is in the maze
# Input: location
# Output: True if in the maze
def in_maze(location):
    if (location[0] >= 0) and (location[0] < maze_size[0]) and (location[1] >= 0) and (location[1] < maze_size[1]):
        return True
    else:
        return False

# Comparing current node with the goal state
# Input: current node and goal node
# Output: Returns true if the nodes are same
def compare_nodes(current_node, goal_node):
    
    if (np.array_equal(current_node, goal_node)):
        return True
    else:
        return False

# Extract the node locations to a list
# Input: list of nodes
# Output: list of the locations
def node_location_extractor (list):
    new_list = []
    for i in list:
        new_list.append(i.center_location)
    return new_list

# Method to get the node with minimum cost
# Input: Open queue
# Output:  the node with mininum cost and it's index
def min_cost_node (list):
    min_node = list[0]
    index = 0

    for a in list:
        if a.cost_to_come < min_node.cost_to_come:
            index = list.index(a)
            min_node = a
    return min_node, index

# Method to update the current nodes based on cost
# Input: Current node and current node list
# Output: updated current node list
def current_node_updater (current_node, list):
    if list:
        future_locations = [[(current_node.center_location[0] - 1, current_node.center_location[1]), 1],
                            [(current_node.center_location[0], current_node.center_location[1] + 1), 1],
                            [(current_node.center_location[0] + 1, current_node.center_location[1]), 1],
                            [(current_node.center_location[0], current_node.center_location[1] - 1), 1],
                            [(current_node.center_location[0] - 1, current_node.center_location[1] + 1), 1.4],
                            [(current_node.center_location[0] + 1, current_node.center_location[1] + 1), 1.4],
                            [(current_node.center_location[0] + 1, current_node.center_location[1] - 1), 1.4],
                            [(current_node.center_location[0] - 1, current_node.center_location[1] - 1), 1.4]]

        for contents in list:
            
            index = 0
            for locations in future_locations:

                if contents.center_location == locations[0]:
                    
                    index = future_locations.index(locations)
                    if (current_node.cost_to_come + locations[1]) < contents.cost_to_come:

                        print('Found better cost in updater')
                    
                        list[index].cost_to_come = np.round(current_node.cost_to_come + locations[1], 2)
                        list[index].parent_index = current_node.node_index
        
        return list

    else:
        return []