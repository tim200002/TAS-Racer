import rclpy
from path_finder.occupancy_grid_listener import OccupancyGridListener
from path_finder.find_path_service import FindPathService


def main(args=None):
    rclpy.init(args=args)

    # start occupancy grid listener
    # print("start occupancy grid listener")
    # occupancy_grid_listener = OccupancyGridListener()
    # rclpy.spin(occupancy_grid_listener)

    # start path planner service
    print("start planner service")
    path_planner_service = FindPathService(simulate_with_loaded_costmap=False)
    rclpy.spin(path_planner_service)

    #! Cleanup
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # occupancy_grid_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
