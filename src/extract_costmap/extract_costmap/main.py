import rclpy
from rclpy.node import Node

from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid

from .numpy_conversion import occupancygrid_to_numpy
import numpy as np


class CostmapExtractor(Node):

    def __init__(self):
        super().__init__('costmap_extractor')
        self.subscription_costmap = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self.listener_callback_costmap,
            10)
        self.subscription_grid = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback_occupancy_grid,
            10)

    def listener_callback_costmap(self, msg):
        return
        #self.get_logger().info('Costmap"%s"' % msg.data)
    
    def listener_callback_occupancy_grid(self, msg):
        numpy_occupancy_grid = occupancygrid_to_numpy(msg)
        print(numpy_occupancy_grid)
        np.save('/home/parallels/ros2_tas_project/to_be_saved/occupancy_grid.npy', numpy_occupancy_grid.data)
        #self.get_logger().info('Grid "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CostmapExtractor()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()