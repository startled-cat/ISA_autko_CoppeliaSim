try:
    from enum import Enum
except:
    print("=================================")
    print("error: cant import enum")
    print("protip: remember to install enum34 for Python 2.7")
    print("(use pip from python 2.7 folder)")
    print("\"pip install enum34\"")
    print("=================================")

class Direction(Enum):
    NORTH = [-1, 0]
    SOUTH = [1, 0]
    EAST = [0, 1]
    WEST = [0, -1]
    UNKNOWN = [0, 0]

    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    BACKWARD = 4

    @staticmethod
    def get_direction_from_angle(angle):
        if angle > 80 and angle < 100:
            return Direction.WEST
        if angle > -100 and angle < -80:
            return Direction.EAST
        if angle < 10 and angle > -10:
            return Direction.NORTH
        if angle > 170 or angle < -170:
            return Direction.SOUTH
        return Direction.UNKNOWN

    @staticmethod
    def get_direction_from_vector(vector):
        if vector == [0, -1]:
            return Direction.WEST
        if vector == [0, 1]:
            return Direction.EAST
        if vector == [-1, 0]:
            return Direction.NORTH
        if vector == [1, 0]:
            return Direction.SOUTH
        return Direction.UNKNOWN
