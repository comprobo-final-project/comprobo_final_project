class Vector2:

    def __init__(self):
        self.x = None
        self.y = None

    def __rmul__(self, other):
        result = Vector2()
        result.x = other * self.x
        result.y = other * self.y
        return result

    def __radd__(self, other):
        result = Vector2()
        result.x = other.x + self.x
        result.y = other.y + self.y
        return result
