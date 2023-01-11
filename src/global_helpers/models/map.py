import numpy as np
from .point import Point, RefPoint

class Map:
    def __init__(self, grid: np.ndarray, resolution_: float, origin_wf: Point):
        self.grid = grid
        self.resolution = resolution_

        self.dimension_x = grid.shape[1]
        self.dimension_y = grid.shape[0]

        # Origin is given
        if origin_wf != None:
            self.origin_wf = origin_wf
        # No origin given, manually calculate origin, for this assume
        # that the origin is the middle point of the map
        else:
            center_point_grid = Point(self.grid.shape[1] / 2, self.grid.shape[1] / 2)
            self.origin_wf = Point(-center_point_grid.x / 2 * resolution_, -center_point_grid.y / 2 * resolution_)

    

    def grid_point_to_world_point(self, point: Point|RefPoint) -> Point|RefPoint:
        # The origin is the bottom left point of the map
        # thus we need some trick when calculating grid (index from top left)
        # to world frame
        x_new = self.origin_wf.x + point.x * self.resolution
        y_new = self.origin_wf.y + point.y * self.resolution

        if type(point) == Point:    
            return Point(x_new, y_new) 
        
        w_tr_right_new = point.w_tr_right * self.resolution
        w_tr_left_new = point.w_tr_left * self.resolution
        return RefPoint(x_new, y_new, w_tr_right_new, w_tr_left_new)
    
    def world_point_to_grid_point(self, point: Point|RefPoint) -> Point|RefPoint:

        if (point.x) < self.origin_wf.x or point.y < self.origin_wf.y:
            raise ValueError("Invalid Coordinates")
        
        x_new = int((point.x - self.origin_wf.x) / self.resolution)
        y_new = int((point.y - self.origin_wf.y) / self.resolution)

        if type(point) == Point:  
            return Point(x_new, y_new)
        w_tr_right_new = point.w_tr_right / self.resolution
        w_tr_left_new = point.w_tr_left / self.resolution
        return RefPoint(x_new, y_new, w_tr_right_new, w_tr_left_new)
    
        
