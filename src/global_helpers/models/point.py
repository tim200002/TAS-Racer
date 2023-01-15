import math
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def norm(self):
        return math.sqrt(self.x**2 + self.y**2)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    
    def __eq__(self, other):
        if other == None:
            return False
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"({self.x},{self.y})"
    
    def __hash__(self):
        return hash((self.x, self.y))

class RefPoint(Point):
    def __init__(self, x, y, w_tr_right, w_tr_left):
        self.w_tr_right = w_tr_right
        self.w_tr_left = w_tr_left
        super().__init__(x, y)

    def __eq__(self, other):
        if other == None:
            return False
        return super().__eq__(other) and  self.w_tr_left == other.w_tr_left and self.w_tr_right == other.w_tr_right

    def __str__(self):
        return f"({self.x},{self.y},{self.w_tr_right},{self.w_tr_left})"
    
    def __hash__(self):
        return hash((self.x, self.y, self.w_tr_right, self.w_tr_left))

    


