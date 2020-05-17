import math
import numpy as np
import time

from direction import Direction
from celltype import CellType


class Map:
    # 1 in maze is place where we have been, 2 is place where is wall
    def __init__(self, width, height, startPos):
        #print("dsdsdsd")
        self.maze = np.zeros(shape=(width, height))
        self.maze[startPos[0], startPos[1]] = CellType.CLEAR.value
        self.currentPosition = startPos
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
                        s = s + "V "
                    elif direction == Direction.WEST.value:
                        s = s + "< "
                    elif direction == Direction.EAST.value:
                        s = s + "> "
                else:
                    cell = self.maze[y][x]
                    if cell == CellType.UNKNOWN.value:
                        s = s + "  "
                    elif cell == CellType.DISCOVERED.value:
                        s = s + "o "
                    elif cell == CellType.BLOCKED.value:
                        s = s + "##"
                    elif cell == CellType.CLEAR.value:
                        s = s + ". "
                    else:
                        s = s + "* "
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
            

