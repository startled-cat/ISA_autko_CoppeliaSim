try:
    from enum import Enum
except:
    print("=================================")
    print("error: cant import enum")
    print("protip: remember to install enum34 for Python 2.7")
    print("(use pip from python 2.7 folder)")
    print("\"pip install enum34\"")
    print("=================================")

class CellType(Enum):
    UNKNOWN = 0 # dont know whats there
    DISCOVERED = 1 # already been on this tile
    BLOCKED = 2 # is wall
    CLEAR = 3 # seen that its clear but never been there

    @staticmethod
    def get_celltype_from_value(value):
        if value == 0 :
            return CellType.UNKNOWN
        if value == 1 :
            return CellType.DISCOVERED
        if value == 2 :
            return CellType.BLOCKED
        if value == 3 :
            return CellType.CLEAR
        return -1
