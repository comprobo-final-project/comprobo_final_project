import math

from vector_3 import Vector3
from polar_vector import PolarVector

class Pose:

    def __init__(self):

        self.position = Vector3()
        self.velocity = PolarVector()
