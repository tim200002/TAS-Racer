import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
import time


class GetEntityStateClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def gazebo_get_model_state(self, name, reference_frame=""):
        get_entity_state_request = GetEntityState.Request()
        get_entity_state_request.reference_frame = reference_frame
        get_entity_state_request.name = name
        get_entity_state_result = self.cli.call_async(get_entity_state_request)
        rclpy.spin_until_future_complete(self, get_entity_state_result)
        original_state = get_entity_state_result.result()
        return original_state

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = GetEntityStateClient()

    time_of_last_request = 0

    while(True):
        response = minimal_client.gazebo_get_model_state(name="waffle_pi", reference_frame="")
        print(response.state.pose)
        time.sleep(1.0)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
