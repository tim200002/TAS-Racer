from models.point import Point

def get_center_line(outer_line1: list[Point], outer_line2: list[Point]):
    middle_line = []
    for point_1 in outer_line1:
        min_distance = 1e12
        min_point = None
        for point_2 in outer_line2:
            distance = (point_1.x - point_2.x)**2 + \
                (point_1.y - point_2.y)**2
            if distance < min_distance:
                min_distance = distance
                min_point = point_2
        middle_x = point_1.x + round(0.5*(min_point.x - point_1.x))
        middle_y = point_1.y + round(0.5*(min_point.y - point_1.y))
        middle_line.append(Point(middle_x, middle_y))

    return middle_line
