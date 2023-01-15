def get_center_line(outer_line1, outer_line2):
    middle_line = []
    for point_1 in outer_line1:
        min_distance = 1e12
        min_point = None
        for point_2 in outer_line2:
            distance = (point_1[0] - point_2[0])**2 + \
                (point_1[1] - point_2[1])**2
            if distance < min_distance:
                min_distance = distance
                min_point = point_2
        middle_x = point_1[0] + round(0.5*(min_point[0] - point_1[0]))
        middle_y = point_1[1] + round(0.5*(min_point[1] - point_1[1]))
        middle_line.append((middle_x, middle_y))

    return middle_line
