from vector_2 import Vector2
from quaternion import Quaternion

class Twist:

    def __init__(self):
        self.linear = Vector2()
        self.angular = Quaternion()
