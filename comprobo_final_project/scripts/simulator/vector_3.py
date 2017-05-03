class Vector3:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


    def __sub__(self, other):
        result = Vector3()
        result.x = self.x - other.x
        result.y = self.y - other.y
        result.z = self.z - other.z
        return result


    def __rmul__(self, other):
        """ Overload right multiplication. """

        result = Vector3()
        result.x = other * self.x
        result.y = other * self.y
        result.z = other * self.z
        return result


    def __radd__(self, other):
        """ Overload right addition. """

        result = Vector3()
        result.x = other.x + self.x
        result.y = other.y + self.y
        result.z = other.z + self.z
        return result


    def __div__(self, other):
        """ Overload left division. """

        result = Vector3()
        result.x = self.x / other
        result.y = self.y / other
        result.z = self.z / other
        return result
