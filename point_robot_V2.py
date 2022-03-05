from inspect import currentframe
import queue
import cv2 as cv
import numpy as np
from maze_explorer_libraries import *
import copy 

# Method for creating the maze
# Input: clearance of the robot
# Output: The maze and the 
def maze_creation (clearance = 0):

    # Shape of the maze
    #maze_shape = (250, 400, 3)
    global maze_shape

    # Creating the maze
    maze = np.zeros(maze_shape, dtype="uint8")

    # The center of the circle
    circle_center = (300, 65)

    # The vertices of the hexagon
    hexagon = np.array([[165, 130], [200, 110], [235, 130], [235, 170], [200, 190], [165, 170]], np.int32)

    # The vertices of the boomerang
    boomerang = np.array([[36, 65], [115, 40], [80, 70], [105, 165]], dtype=np.int32)

    # Creating the shapes
    for rows in range(maze_shape[0]):
        for columns in range(maze_shape[1]):

            # Creating the circle
            circle_value = (rows - circle_center[1]) ** 2 + (columns - circle_center[0]) ** 2 - (40 + clearance) ** 2 

            if (circle_value <= 0):
                maze[rows, columns, 0] = 255

            # Creating the boomerang
            # Triangle 1 of the boomerang
            line_1 = (rows) + (0.37 * columns) + (-76.4 + clearance)
            line_2 = (rows) + (-0.114 * columns) + (-60.91)
            line_3 = (rows) + (0.86 * columns) + (-138.6 - clearance)

            if (line_1 > 0) and (line_2 < 0) and (line_3 < 0):
                maze[rows, columns, 0] = 255

            # Triangle 2 of the boomerang
            line_1 = (rows) + (-1.232 * columns) + (-20.65 - clearance)
            line_2 = (rows) + (-0.114 * columns) + (-60.91)
            line_3 = (rows) + (-3.2 * columns) + (186 + clearance)

            if (line_1 < 0) and (line_2 > 0) and (line_3 > 0):
                maze[rows, columns, 0] = 255

            # Creating the hexagon
            line_1 = (rows) + (0.57 * columns) + (-224.3 + clearance)
            line_2 = (rows) + (-0.57 * columns) + (4.29 + clearance)
            line_3 = (columns) + (-235 - clearance)
            line_4 = (rows) + (0.57 * columns) + (-304.29 - clearance)
            line_5 = (rows) + (-0.57 * columns) + (-75.71 - clearance)
            line_6 = (columns) + (-165 + clearance)

            if (line_1 > 0) and (line_2 > 0) and (line_3 < 0) and (line_4 < 0) and (line_5 < 0) and (line_6 > 0):
                maze[rows, columns, 0] = 255
    
    # Checking the created maze
    cv.imshow('maze', maze)
    cv.waitKey(0)

    # Finding the points where we have the obstacles
    x_coordinates, y_coordinates = np.where(maze[:,:,0] == 255)
    obstacle_space = []
    for x, y in zip(x_coordinates, y_coordinates):
        obstacle_space.append((x, y))
    # print(obstacle_space)

    return maze, obstacle_space


# Method for getting the user input
# Input: None
# Ouput: The initial and goal node entered by the user
def user_input (default_init, default_goal):
    
    user = 'y'
    initial_flag = False
    input_complete = False
    
    while (user == 'y' and not input_complete):

        user  = input('Would you like to enter a custom initial and goal node? (y/n) ')
        user = user[0].lower()
        
        if (user == 'y'):

            while (not input_complete):
                if (not initial_flag):
                    initial_node = input('Enter the initial node: ').replace(',', ' ').split()
                    initial_node = int(initial_node[0]), int(initial_node[1])
                    print('The entered initial node location is: ', initial_node)
                    if (found_node(initial_node, obstacle_space) or not in_maze(initial_node)):
                        print('Entered location is in the obstacle space or not in the maze!\nPlease try again!')
                        continue
                    else:
                        initial_flag = True
                else:
                    goal_node = input('Enter the goal node: ').replace(',', ' ').split()
                    goal_node = int(goal_node[0]), int(goal_node[1])
                    print('The entered goal node location is: ', goal_node)
                    if (found_node(goal_node, obstacle_space) or not in_maze(goal_node)):
                        print('Entered location is in the obstacle space or not in the maze!\nPlease try again!')
                        continue
                    else:
                        input_complete = True
        else:
            print(f'The default initial node is {default_init} and default the goal is {default_goal}')
            initial_node, goal_node = default_init, default_goal
            break
    return initial_node, goal_node

if __name__ == '__main__':

    # Cost map for each direction in the following order [North, East, South, West, North-East, North-West, South-East, South-West]
    cost_map = (1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4)
    
    # Shape of the maze
    global maze_shape
    maze_shape = (250, 400, 3)
    
    # Creating the maze with clearance of 5
    maze, obstacle_space = maze_creation(5)
    
    # Setting initial and goal nodes
    # initial_node = (249, 0)
    # goal_node = (0, 399)
    initial_node = (0, 0)
    goal_node = (2, 2)

    # print(found_node(initial_node, obstacle_space))
    
    # Getting the user input
    try:
        initial_node, goal_node = user_input(initial_node, goal_node)
    except:
        print(f'There is an error in the user input!\nUsing the default nodes....\nInitial node : {initial_node} and Goal node : {goal_node}')

    '''Start of Dijkstra Algorithm'''
    
    # Maintaining the node index
    node_index = 0
    
    # Creating the queues
    open_queue  = []
    visited_queue = []

    # Goal flag to be raised when goal is found
    goal_node_reached = False

    # Creating the current node
    current_Node = Node()

    # Updating the current node with the initial node values
    current_Node.update_node(initial_node, node_index, node_location_extractor(visited_queue), node_location_extractor(open_queue), obstacle_space)
    node_index += 1
    current_Node.node_index = node_index
    # current_Node.print_current_values()

    # Appending the current node to the open queue
    open_queue.append(current_Node)

    # Creating a temporary node to keep track of the new created nodes
    temp_Node = Node()

    # Threshold to stop infinite loops
    thresh = maze_shape[0] * maze_shape[1]
    #print('Threshold: ', thresh)

    # Index for popping the visited
    index_to_pop = 0

    while not goal_node_reached:

        # Quit if there are no child nodes
        # if current_Node.child_nodes == 0:
        #     print("Robot is stuck, Nowhere to go!")
        #     goal_node_reached = True
        #     break

        # For every possible action create a new node and update the queue
        for i in range(len(current_Node.possible_actions)):
            
            open_queue = current_node_updater(current_Node, open_queue)

            # Creating the child nodes
            if (current_Node.possible_actions[i]):

                temp_Node = copy.deepcopy(current_Node)
                #current_Node = current_node_updater(temp_Node, current_Node)
                
                if (i == 0):
                    temp_Node.move_north(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 1):
                    temp_Node.move_east(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 2):
                    temp_Node.move_south(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 3):
                    temp_Node.move_west(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 4):
                    temp_Node.move_north_east(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 5):
                    temp_Node.move_south_east(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 6):
                    temp_Node.move_south_west(current_Node.center_location, current_Node.cost_to_come, cost_map[i])
                elif (i == 7):
                    temp_Node.move_north_west(current_Node.center_location, current_Node.cost_to_come, cost_map[i])

                temp_Node.update_node(temp_Node.center_location, current_Node.node_index, node_location_extractor(visited_queue), node_location_extractor(open_queue), obstacle_space)
                node_index += 1
                temp_Node.node_index = node_index

                # To be removed
                # print('\n')
                # temp_Node.print_current_values()

                # Appending to the open queue
                open_queue.append(temp_Node)

                if (temp_Node.center_location == goal_node):
                    print("\nFound the node")
                    print("==============\n")
                    goal_node_reached = True
                    break
        
        visited_queue.append(open_queue.pop(index_to_pop))
        # To be deleted later
        if goal_node_reached:
            visited_queue.append(temp_Node)
        # current_Node = copy.copy(open_queue[0])
        current_Node, index_to_pop = min_cost_node(open_queue)
        current_Node.action_checker(node_location_extractor(visited_queue), node_location_extractor(open_queue), obstacle_space)

        # Handling the impossible scenarios
        thresh -= 1
        if thresh == 0:
            break
            # return -1
    
    print("The Art:")
    for every_node in visited_queue:
        print(f'Node index is {every_node.node_index} parent is {every_node.parent_index} center is: {every_node.center_location} with cost {every_node.cost_to_come}')