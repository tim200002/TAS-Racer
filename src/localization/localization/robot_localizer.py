import string
import rclpy
from rclpy.node import Node
from .utils.coordinate_transforms import invert_transform, multiply_transform

from geometry_msgs.msg import Transform, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_ros import TransformBroadcaster


def print_transform(A: Transform):
    return f"({A.translation.x}, {A.translation.y}, {A.translation.z}), ({A.rotation.x},{A.rotation.y},{A.rotation.z},{A.rotation.w})"


class Localizer(Node):
    def __init__(self):
        super().__init__(node_name='frame_listener')
        
        # how often to publish new transforms
        self.publish_frequency = 50
        
        # how often to ask for new transforms
        # setting this higher did not achieve better performance
        self.tf_update_frequency = 10

        # For TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_update_timer = self.create_timer(1/self.tf_update_frequency, self._get_tf2_state)

        # For geeting footprint location updates
        topic = "gps/odom"
        self.subscriber = self.create_subscription(Odometry, topic, self._on_location_update, 10)

        # for calculating frame transforms
        self.transform_publish_timer = self.create_timer(1/self.publish_frequency, self._publish_map_odom_transform)
        
        # transform from map to footprint (i.e. car)
        self.map_footprint_transform = None
        # transfrom from odom to footprint (i.e. car)
        self.odom_fottprint_transform = None

        # for broadcasting tf updates
        self.tf_broadcaster = TransformBroadcaster(self)



    def _on_location_update(self, msg: Odometry):
        new_transform = Transform()
        new_translation = Vector3()
        new_translation.x = msg.pose.pose.position.x
        new_translation.y = msg.pose.pose.position.y
        new_translation.z = msg.pose.pose.position.z
        new_transform.translation = new_translation
        
        new_transform.rotation = msg.pose.pose.orientation
        
        self.map_footprint_transform = new_transform

    def _get_transform(self, referece_frame: string, target_frame: string):
        try:
            t = self.tf_buffer.lookup_transform(
                referece_frame,
                target_frame,
                rclpy.time.Time())
            return t
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {referece_frame} to {target_frame}: {ex}')
            return

    def _get_tf2_state(self):
        self.odom_fottprint_transform = self._get_transform("odom", "base_footprint")
        if(self.odom_fottprint_transform):
            self.odom_fottprint_transform = self.odom_fottprint_transform.transform

    def _publish_map_odom_transform(self):
        if self.odom_fottprint_transform == None:
            self.get_logger().info("Odom footprint transform None")
            return
        if(self.map_footprint_transform == None):
            self.get_logger().info("Map footprint transform is None")
            return

        map_odom_transform = multiply_transform(self.map_footprint_transform, invert_transform(self.odom_fottprint_transform)) 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform = map_odom_transform
        self.tf_broadcaster.sendTransform(t)    


def main(args=None):
    rclpy.init(args=args)
    node = Localizer()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
   