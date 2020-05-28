import math
import numpy as np
import time

from direction import Direction
from celltype import CellType
from node import Node



def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        #print("Next Node")
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # print("OPEN LIST:")
        # for node in open_list:
        #     print(node.position)
        # time.sleep(1)
        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = [current_node.position[0] + new_position[0], current_node.position[1] + new_position[1]]
            
            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != CellType.DISCOVERED.value:
                continue

            # Create new node
            new_node = Node(current_node, node_position)
            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            def check_children():
                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        return
                    
                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        return

                # Add the child to the open list
                open_list.append(child)

            check_children()





def set_map_for_testing():
        self.maze = [[CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value]]




def main():

    maze = [[CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value],
            [CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.UNKNOWN.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.DISCOVERED.value, CellType.BLOCKED.value, CellType.BLOCKED.value, CellType.UNKNOWN.value],
            [CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value, CellType.UNKNOWN.value]]
    print(maze[8][6])
    print(maze[1][8])

    start = (8, 6)
    end = (1, 8)

    path = astar(maze, start, end)
    print(path)


if __name__ == '__main__':
    main()
