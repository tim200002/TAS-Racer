import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from .numpy_conversion import occupancy_grid_to_numpy
import yaml


class OccupancyGridListener(Node):
    def __init__(self):
        super().__init__('occupancy_grid_listener')

        self.subscription_grid = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback_occupancy_grid,
            10)

    def listener_callback_occupancy_grid(self, msg: OccupancyGrid):
        self.get_logger().info('New global costmap received updating shared object 2')

        # store costmap for debugging purposes
        numpy_occupancy_grid = occupancy_grid_to_numpy(msg)
        np.save('/home/tim/tas2-racer/to_be_saved/costmap/occupancy_grid.npy',
                numpy_occupancy_grid.data)

        data = {
            "origin_x": msg.info.origin.position.x,
            "origin_y": msg.info.origin.position.y,
            "resolution": msg.info.resolution
        }

        with open('/home/tim/tas2-racer/to_be_saved/costmap/occupancy_grid_info.yaml', 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
