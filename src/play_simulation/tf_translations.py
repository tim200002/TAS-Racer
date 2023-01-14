import string
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Transform, Vector3, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations


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
    #print(trans1)
    trans1 = tf_transformations.translation_matrix(trans1)
    #print(trans1)
    rot1 = A.rotation
    rot1 = np.array([rot1.x, rot1.y, rot1.z, rot1.w])
    rot1 = tf_transformations.quaternion_matrix(rot1)
    mat1 = tf_transformations.concatenate_matrices(trans1, rot1)
    #print(mat1)

    # print("B")
    # print(B)
    trans2 = B.translation
    # print(trans2.x)
    trans2 = np.array([trans2.x, trans2.y, trans2.z])

    trans2 = tf_transformations.translation_matrix(trans2)

    rot2 = B.rotation
    rot2 = np.array([rot2.x, rot2.y, rot2.z, rot2.w])
    rot2 = tf_transformations.quaternion_matrix(rot2)
    mat2 = tf_transformations.concatenate_matrices(trans2, rot2)
    #print(mat2)

    # print(mat1)
    # print(mat2)

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
        super().__init__('frame_listener')


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

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
            print(
                f'Could not transform {referece_frame} to {target_frame}: {ex}')
            return

    def on_timer(self):
        map_odom = None
        odom_map = None
        odom_footprint = None
        map_footprint = None
        
        map_odom = self.get_transform("map", "odom")
        odom_map = self.get_transform("odom", "map")
        odom_footprint = self.get_transform("odom", "base_footprint")
        map_footprint = self.get_transform("map", "base_footprint")

        if map_odom and odom_footprint and map_footprint and odom_footprint:
            print("map odom")
            print(map_odom.transform)
            print("odom map")
            print(odom_map.transform)
            print("odom footprint")
            print(odom_footprint.transform)
            print("map footprint")
            print(map_footprint.transform)

            print("map footprint multiplied")
            map_footprint_multiplied = multiply_transform(map_odom.transform, odom_footprint.transform)
            print(print_transform(map_footprint_multiplied))

            print("map odom inverted")
            map_odom_inverted = invert_transform(map_odom.transform)
            print(print_transform(map_odom_inverted))

            print("map odom calculated")
            map_odom_calculated = multiply_transform(map_footprint.transform, invert_transform(odom_footprint.transform))
            print(print_transform(map_odom_calculated))

            


if __name__ == '__main__':
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()