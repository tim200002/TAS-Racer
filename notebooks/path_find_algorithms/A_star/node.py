from models.point import Point

class Node:
    def __init__(self, coordinate: Point, f_score):
        self.coordinate = coordinate
        self.f_score = f_score

    def __lt__(self, other):
        return self.f_score < other.f_score
