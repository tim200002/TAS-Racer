import tf_transformations
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy as np

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
    # convert transform 1 in right format
    trans1 = A.translation
    trans1 = np.array([trans1.x, trans1.y, trans1.z])
    trans1 = tf_transformations.translation_matrix(trans1)
    rot1 = A.rotation
    rot1 = np.array([rot1.x, rot1.y, rot1.z, rot1.w])
    rot1 = tf_transformations.quaternion_matrix(rot1)
    mat1 = tf_transformations.concatenate_matrices(trans1, rot1)

    # convert transform 2 in right format
    trans2 = B.translation
    trans2 = np.array([trans2.x, trans2.y, trans2.z])
    trans2 = tf_transformations.translation_matrix(trans2)
    rot2 = B.rotation
    rot2 = np.array([rot2.x, rot2.y, rot2.z, rot2.w])
    rot2 = tf_transformations.quaternion_matrix(rot2)
    mat2 = tf_transformations.concatenate_matrices(trans2, rot2)
    
    # add transforms together
    mat3 = np.matmul(mat1, mat2)

    trans3 = tf_transformations.translation_from_matrix(mat3)
    rot3 = tf_transformations.quaternion_from_matrix(mat3)

    # create tranform message 
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