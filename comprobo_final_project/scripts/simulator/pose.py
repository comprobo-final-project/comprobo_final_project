from vector_3 import Vector3
from quaternion import Quaternion

class Pose:

    def __init__(self):
        self.position = Vector3()
        self.orientation = Quaternion()
