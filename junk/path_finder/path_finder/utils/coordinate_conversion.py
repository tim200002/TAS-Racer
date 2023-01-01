from nav_msgs.msg import OccupancyGrid


def world_frame_point_to_occupancy_grid_coordinates(point, occupancy_grid: OccupancyGrid):
    point_x = int((point[0] - occupancy_grid.info.origin.position.x) /
                  occupancy_grid.info.resolution)

    point_y = int((point[1] - occupancy_grid.info.origin.position.y) /
                  occupancy_grid.info.resolution)

    return (point_x, point_y)


def occupancy_grid_coordinates_to_world_frame_point(point, occupancy_grid: OccupancyGrid):
    point_x = (point[0] * occupancy_grid.info.resolution) + \
        occupancy_grid.info.origin.position.x
    point_y = (point[1] * occupancy_grid.info.resolution) + \
        occupancy_grid.info.origin.position.y
    return (point_x, point_y)
