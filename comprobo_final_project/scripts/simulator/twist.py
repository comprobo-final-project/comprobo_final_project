"""
Custom made Twist message attribute for simulation
"""

from vector_3 import Vector3
from quaternion import Quaternion


class Twist:

    def __init__(self):

        self.linear = Vector3()
        self.angular = Vector3()
