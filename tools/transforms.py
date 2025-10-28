import numpy as np

def get_rotation_z(theta):
    """Returns a 4x4 rotation matrix around the Z-axis."""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

def get_translation_z(d):
    """Returns a 4x4 translation matrix along the Z-axis."""
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

def get_translation_x(a):
    """Returns a 4x4 translation matrix along the X-axis."""
    return np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def get_rotation_x(alpha):
    """Returns a 4x4 rotation matrix around the X-axis."""
    return np.array([
        [1, 0,             0,              0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha),  np.cos(alpha), 0],
        [0, 0,             0,              1]
    ])

def create_transformation_matrix(translation, rotation_matrix):
    """
    Creates a 4x4 transformation matrix from a translation vector and a rotation matrix.
    """
    T = np.identity(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T