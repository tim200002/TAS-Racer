import string
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from tf2_ros import TransformBroadcaster


def print_transform(A: Transform):
    return f"({A.translation.x}, {A.translation.y}, {A.translation.z}), ({A.rotation.x},{A.rotation.y},{A.rotation.z},{A.rotation.w})"

def invert_transform(transform: Transform):
    trans = transform.translation
    trans = np.array([trans.x, trans.y, trans.z])
    rot = transform.rotation
    rot = np.array([rot.x, rot.y, -rot.z, -rot.w])

    transform = tf_transformations.concatenate_matrices(tf_transformations.translation_matrix(trans), tf_transformations.quaternion_matrix(rot))
    inverse = tf_transformations.inverse_matrix(transform)


    translation_inv = tf_transformations.translation_from_matrix(inverse)
    rotation_inv = tf_transformations.quaternion_from_matrix(inverse)

    new_translation = Vector3()
    new_translation.x = translation_inv[0]
    new_translation.y = translation_inv[1]
    new_translation.z = translation_inv[2]

    new_rotation = Quaternion()
    new_rotation.x = rotation_inv[0]
    new_rotation.y = rotation_inv[1]
    new_rotation.z = rotation_inv[2]
    new_rotation.w = rotation_inv[3]

    new_transform = Transform()
    new_transform.translation = new_translation
    new_transform.rotation = new_rotation

    return new_transform

def multiply_transform(A: Transform, B: Transform):
    trans1 = A.translation
    trans1 = np.array([trans1.x, trans1.y, trans1.z])
    #self.get_logger().info(trans1)
    trans1 = tf_transformations.translation_matrix(trans1)
    #self.get_logger().info(trans1)
    rot1 = A.rotation
    rot1 = np.array([rot1.x, rot1.y, rot1.z, rot1.w])
    rot1 = tf_transformations.quaternion_matrix(rot1)
    mat1 = tf_transformations.concatenate_matrices(trans1, rot1)
    #self.get_logger().info(mat1)

    # self.get_logger().info("B")
    # self.get_logger().info(B)
    trans2 = B.translation
    # self.get_logger().info(trans2.x)
    trans2 = np.array([trans2.x, trans2.y, trans2.z])

    trans2 = tf_transformations.translation_matrix(trans2)

    rot2 = B.rotation
    rot2 = np.array([rot2.x, rot2.y, rot2.z, rot2.w])
    rot2 = tf_transformations.quaternion_matrix(rot2)
    mat2 = tf_transformations.concatenate_matrices(trans2, rot2)
    #self.get_logger().info(mat2)

    # self.get_logger().info(mat1)
    # self.get_logger().info(mat2)

    mat3 = np.matmul(mat1, mat2)


    trans3 = tf_transformations.translation_from_matrix(mat3)
    rot3 = tf_transformations.quaternion_from_matrix(mat3)

    new_translation = Vector3()
    new_translation.x = trans3[0]
    new_translation.y = trans3[1]
    new_translation.z = trans3[2]

    new_rotation = Quaternion()
    new_rotation.x = rot3[0]
    new_rotation.y = rot3[1]
    new_rotation.z = rot3[2]
    new_rotation.w = rot3[3]

    new_transform = Transform()
    new_transform.translation = new_translation
    new_transform.rotation = new_rotation


    return new_transform

    # new_transform = Transform
    # new_transform.translation = translation_inv
    # new_transform.rotation = rotation_inv


class FrameListener(Node):

    def __init__(self):
        super().__init__(node_name='frame_listener')
        self.get_logger().info(str(self.get_clock().now().seconds_nanoseconds()[0]))
        # For TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_update_timer = self.create_timer(1.0, self.get_tf2_state)

        # For geeting car updates
        topic = "gps/odom"
        self.subscriber = self.create_subscription(Odometry, topic, self.on_location_update, 10)

        # for calculating frame transforms
        self.transform_publish_timer = self.create_timer(1.0, self.publish_map_odom_transform)
        self.map_car_transform = None
        self.odom_car_transform = None

        # for broadcasting tf updates
        self.tf_broadcaster = TransformBroadcaster(self)



    def on_location_update(self, msg: Odometry):
        new_transform = Transform()
        new_translation = Vector3()
        new_translation.x = msg.pose.pose.position.x
        new_translation.y = msg.pose.pose.position.y
        new_translation.z = msg.pose.pose.position.z
        new_transform.translation = new_translation
        
        new_transform.rotation = msg.pose.pose.orientation
        
        self.map_car_transform = new_transform

    def get_transform(self, referece_frame: string, target_frame: string):
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
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

    def get_tf2_state(self):
        self.odom_car_transform = self.get_transform("odom", "base_footprint")
        if(self.odom_car_transform):
            self.odom_car_transform = self.odom_car_transform.transform

    def publish_map_odom_transform(self):
        if self.odom_car_transform == None:
            self.get_logger().info("Odom car transform None")
            return
        if(self.map_car_transform == None):
            self.get_logger().info("Map car transform is None")
            return

        map_odom_transform = multiply_transform(self.map_car_transform, invert_transform(self.odom_car_transform)) 
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform = map_odom_transform
        self.tf_broadcaster.sendTransform(t)    

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
   