import math

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
        self._w = (val + math.pi) % (2*math.pi) - math.pi


    @property
    def x(self):
        return self.r * math.cos(self.w)


    @property
    def y(self):
        return self.r * math.sin(self.w)
