import sys, os
#base_path = os.path.split(os.path.join(os.path.dirname(os.path.abspath(__file__))))[0]
base_path = "/home/tim/tas2-racer/src"
sys.path.append(base_path)

import rclpy
from dynamic_path_finder.PathFinder import PathFinder

def main(args=None):
    rclpy.init(args=args)


    # start path planner service
    print("start planner service")
    path_planner_service = PathFinder()
    rclpy.spin(path_planner_service)

    #! Cleanup
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupancy_grid_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
