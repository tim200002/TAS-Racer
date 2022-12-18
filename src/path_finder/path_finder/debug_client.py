from custom_interfaces.srv import FindPath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PathFinderDebugClient(Node):

    def __init__(self):
        super().__init__('path_finder_debug_client')
        self.cli = self.create_client(FindPath, 'find_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FindPath.Request()

    def send_request(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        self.req.start_pose = start_pose
        self.req.goal_pose = goal_pose
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    initial_pose = PoseStamped()
    initial_pose.pose.position.x = -3.0
    initial_pose.pose.position.y = 5.0

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 7.0

    minimal_client = PathFinderDebugClient()
    response = minimal_client.send_request(initial_pose, goal_pose)
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
