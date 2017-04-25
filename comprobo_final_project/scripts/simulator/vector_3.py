class Vector3:

    def __init__(self):

        self.x = 0
        self.y = 0
        self.z = 0


    def __rmul__(self, other):
        """ Overwrite right multiplication. """

        result = Vector3()
        result.x = other * self.x
        result.y = other * self.y
        result.z = other * self.z
        return result


    def __radd__(self, other):
        """ Overwrite right addition. """

        result = Vector3()
        result.x = other.x + self.x
        result.y = other.y + self.y
        result.z = other.z + self.z
        return result
