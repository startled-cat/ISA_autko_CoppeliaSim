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
    UNDISCOVERED = 0
    CLEAR = 1
    BLOCKED = 2
