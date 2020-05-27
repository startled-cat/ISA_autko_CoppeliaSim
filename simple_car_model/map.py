import math
import numpy as np
import time

from node import Node
from direction import Direction
from celltype import CellType


class Map:
    # 1 in maze is place where we have been, 2 is place where is wall
    def __init__(self, width, height, startPos):
        #print("dsdsdsd")
        self.endPosition = (1, 8)
        self.startPosition = startPos
        self.maze = np.zeros(shape=(width, height))
        self.maze[startPos[0], startPos[1]] = CellType.CLEAR.value
        self.currentPosition = startPos
        self.ridingFromPointToPoint = False
        self.path = []
        self.currentPathIndex = 0
        #print(self.maze)

    def print_pretty(self, direction):
        print("------------------------")
        y_max, x_max = self.maze.shape
        for y in range(0, y_max):
            s = "| "
            for x in range(0, x_max):
                if x == self.currentPosition[1] and y == self.currentPosition[0]:
                    if direction == Direction.NORTH.value:
                        s = s + "^ "
                    elif direction == Direction.SOUTH.value:
                        s = s + "v "
                    elif direction == Direction.WEST.value:
                        s = s + "< "
                    elif direction == Direction.EAST.value:
                        s = s + "> "
                else:
                    cell = self.maze[y][x]
                    if cell == CellType.UNKNOWN.value:
                        s = s + "  "
                    elif cell == CellType.DISCOVERED.value:
                        s = s + "+ "
                    elif cell == CellType.BLOCKED.value:
                        s = s + "# "
                    elif cell == CellType.CLEAR.value:
                        s = s + ". "
                    else:
                        s = s + "? "
            s = s + "| "
            print(s)

        print("------------------------")

    def is_all_discovered(self):
        y_max, x_max = self.maze.shape
        for y in range(0, y_max):
            for x in range(0, x_max):
                cell = self.maze[y][x]
                if cell == CellType.CLEAR.value:
                    return False
        return True

    def finished_ride(self):
        return self.ridingFromPointToPoint;

    def set_forward_value(self, direction):# set forward wall as blocked
        # when go up
        if direction[0] == -1:
            self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.BLOCKED.value
        # when go down
        if direction[0] == 1:
            self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.BLOCKED.value
        # when go left
        if direction[1] == -1:
            self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.BLOCKED.value
        # when go right
        if direction[1] == 1:
            self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.BLOCKED.value

    def update_position(self, direction):#moves 'cursor'
        if direction == Direction.NORTH.value:
            #print("UP")
            self.currentPosition[0] = self.currentPosition[0] - 1
        elif direction == Direction.SOUTH.value:
            #print("DOWN")
            self.currentPosition[0] = self.currentPosition[0] + 1
        elif direction == Direction.EAST.value:
            #print("RIGHT")
            self.currentPosition[1] = self.currentPosition[1] + 1
        elif direction == Direction.WEST.value:
            #print("LEFT")
            self.currentPosition[1] = self.currentPosition[1] - 1

    def set_values(self, direction, leftSensorDistance, rightSensorDistance, frontSensorDistance):
        print("Hello Its me, from the other side, updating map ...")
        print("direction = " + str(Direction.get_direction_from_vector(direction)))
        print("leftSensorDistance = " + str(leftSensorDistance))
        print("frontSensorDistance = " + str(frontSensorDistance))
        print("rightSensorDistance = " + str(rightSensorDistance))
        # when go up
        if direction[0] == -1:
            #set first values
            #print("UP")
            #self.currentPosition[0] = self.currentPosition[0] - 1
            self.maze[self.currentPosition[0], self.currentPosition[1]] = CellType.DISCOVERED.value
            
            if (frontSensorDistance < 0):#front side is clear
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.BLOCKED.value

            if (leftSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.BLOCKED.value

            if (rightSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.BLOCKED.value

        #when go down
        if direction[0] == 1:
            #set first values
            #print("DOWN")
            #self.currentPosition[0] = self.currentPosition[0] + 1
            self.maze[self.currentPosition[0], self.currentPosition[1]] = CellType.DISCOVERED.value

            if (frontSensorDistance < 0):#front side is clear
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.BLOCKED.value

            if (leftSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.BLOCKED.value

            if (rightSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.BLOCKED.value
           
        #when go right
        if direction[1] == 1:
            #set first values
            #print("RIGHT")
            #self.currentPosition[1] = self.currentPosition[1] + 1
            self.maze[self.currentPosition[0], self.currentPosition[1]] = CellType.DISCOVERED.value

            if (frontSensorDistance < 0):#front side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]+1] = CellType.BLOCKED.value

            if (leftSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.BLOCKED.value

            if (rightSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.BLOCKED.value

           
        #when go left
        if direction[1] == -1:
            #set first values
            #print("LEFT")
            #self.currentPosition[1] = self.currentPosition[1] - 1
            self.maze[self.currentPosition[0], self.currentPosition[1]] = CellType.DISCOVERED.value

            if (frontSensorDistance < 0):#front side is clear
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0], self.currentPosition[1]-1] = CellType.BLOCKED.value

            if (leftSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]+1, self.currentPosition[1]] = CellType.BLOCKED.value

            if (rightSensorDistance < 0):#left side is clear
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] != CellType.DISCOVERED.value:
                    self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.CLEAR.value
            else:
                self.maze[self.currentPosition[0]-1, self.currentPosition[1]] = CellType.BLOCKED.value
           

        self.print_pretty(direction)

    def check(self, direction_facing, direction_checking):
        cell_value = -1
        if direction_facing == Direction.NORTH.value:#facing up
            if direction_checking == Direction.FORWARD:
                cell_value = self.maze[self.currentPosition[0]-1, self.currentPosition[1]]
            if direction_checking == Direction.LEFT:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]-1]
            if direction_checking == Direction.RIGHT:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]+1]

        if direction_facing == Direction.SOUTH.value:#facing down
            if direction_checking == Direction.FORWARD:
                cell_value = self.maze[self.currentPosition[0]+1, self.currentPosition[1]]
            if direction_checking == Direction.LEFT:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]+1]
            if direction_checking == Direction.RIGHT:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]-1]

        if direction_facing == Direction.WEST.value:#facing left
            if direction_checking == Direction.FORWARD:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]-1]
            if direction_checking == Direction.LEFT:
                cell_value = self.maze[self.currentPosition[0]+1, self.currentPosition[1]]
            if direction_checking == Direction.RIGHT:
                cell_value = self.maze[self.currentPosition[0]-1, self.currentPosition[1]]

        if direction_facing == Direction.EAST.value:#facing right
            if direction_checking == Direction.FORWARD:
                cell_value = self.maze[self.currentPosition[0], self.currentPosition[1]+1]
            if direction_checking == Direction.LEFT:
                cell_value = self.maze[self.currentPosition[0]-1, self.currentPosition[1]]
            if direction_checking == Direction.RIGHT:
                cell_value = self.maze[self.currentPosition[0]+1, self.currentPosition[1]]
        return CellType.get_celltype_from_value(cell_value)
            

    # return 1 if can go front, 2 if can go left, 3 if can go right
    def check_value(self, direction, frontSensorDistance, leftSensorDistance, rightSensorDistance):
        # when go up
        if direction[0] == -1:# direction == Direction.NORTH
            if frontSensorDistance < 0.001 or frontSensorDistance > 1:
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == CellType.CLEAR.value:
                    return 1
            if leftSensorDistance < 0.001 or leftSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == CellType.CLEAR.value:
                    return 2
            if rightSensorDistance < 0.001 or rightSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == CellType.CLEAR.value:
                    return 3
        # when go down
        if direction[0] == 1:# direction == Direction.SOUTH
            if frontSensorDistance < 0.001 or frontSensorDistance > 1:
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == CellType.CLEAR.value:
                    return 1
            if leftSensorDistance < 0.001 or leftSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == CellType.CLEAR.value:
                    return 2
            if rightSensorDistance < 0.001 or rightSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == CellType.CLEAR.value:
                    return 3
        # when go left
        if direction[1] == -1:# direction == Direction.WEST
            if frontSensorDistance < 0.001 or frontSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]-1] == 0:
                    return 1
            if leftSensorDistance < 0.001 or leftSensorDistance > 1:
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == 0:
                    return 2
            if rightSensorDistance < 0.001 or rightSensorDistance > 1:
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == 0:
                    return 3
        # when go right
        if direction[1] == 1:# direction == Direction.EAST
            if frontSensorDistance < 0.001 or frontSensorDistance > 1:
                if self.maze[self.currentPosition[0], self.currentPosition[1]+1] == 0:
                    return 1
            if leftSensorDistance < 0.001 or leftSensorDistance > 1:
                if self.maze[self.currentPosition[0]-1, self.currentPosition[1]] == 0:
                    return 2
            if rightSensorDistance < 0.001 or rightSensorDistance > 1:
                if self.maze[self.currentPosition[0]+1, self.currentPosition[1]] == 0:
                    return 3

    def set_map_for_testing(self):
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

    def check_direction_for_ride(self, direction_facing):
        if self.path[self.currentPathIndex][0] == self.endPosition[0] and self.path[self.currentPathIndex][0] == self.endPosition[1]:
            self.ridingFromPointToPoint = False
            print("End of journey")
            return
        if direction_facing == Direction.NORTH.value:#facing up
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]-1:
                return Direction.FORWARD
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]-1:
                return Direction.LEFT
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]+1:
                return Direction.RIGHT

        if direction_facing == Direction.SOUTH.value:#facing down
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]+1:
                return Direction.FORWARD
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]+1:
                return Direction.LEFT
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]-1:
                return Direction.RIGHT

        if direction_facing == Direction.WEST.value:#facing left
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]-1:
                return Direction.FORWARD
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]+1:
                return Direction.LEFT
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]-1:
                return Direction.RIGHT

        if direction_facing == Direction.EAST.value:#facing right
            if self.path[self.currentPathIndex][1] == self.path[self.currentPathIndex][1]+1:
                return Direction.FORWARD
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]-1:
                return Direction.LEFT
            if self.path[self.currentPathIndex][0] == self.path[self.currentPathIndex][0]+1:
                return Direction.RIGHT
    
    def astar(self, start, end):
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
            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)
            # Found the goal
            if current_node == end_node:
                self.currentPathIndex = 0
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
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                
                # Make sure within range
                if node_position[0] > (len(self.maze) - 1) or node_position[0] < 0 or node_position[1] > (len(self.maze[len(self.maze)-1]) -1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if self.maze[node_position[0]][node_position[1]] != CellType.DISCOVERED.value:
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

                

