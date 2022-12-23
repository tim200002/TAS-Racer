from .point import Point

class Pose:
    def __init__(self, coordinate: Point, angle: float):
        self.coordinate = coordinate
        self.angle = angle

    def __eq__(self, other):
        return self.coordinate == other.coordinate and self.angle == other.angle

    def __str__(self):
        return f"({self.coordinate.x}, {self.coordinate.y}, {self.angle})"