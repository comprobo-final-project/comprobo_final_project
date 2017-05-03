import numpy as np
from vector_3 import Vector3

class PolarVector(object):

    def __init__(self):
        self.r = 0
        self._w = 0


    def __str__(self):
        return str(self.r) + ", " + str(self.w)


    @property
    def w(self):
        return self._w


    @w.setter
    def w(self, val):
        self._w = (val + np.pi) % (2*np.pi) - np.pi


    @property
    def x(self):
        return self.r * np.cos(self.w)


    @property
    def y(self):
        return self.r * np.sin(self.w)


    def to_vector_3(self):
        vector = Vector3()
        vector.x = self.x
        vector.y = self.y
        return vector
