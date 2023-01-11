import rclpy

from .occupancy_grid_listener import OccupancyGridListener



       


def main(args=None):
    rclpy.init(args=args)

    print("start costmap extractor")
    occupancy_grid_listener = OccupancyGridListener()
    rclpy.spin(occupancy_grid_listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main()