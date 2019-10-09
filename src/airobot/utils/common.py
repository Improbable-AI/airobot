from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import PyKDL as kdl
import numpy as np
from scipy.spatial.transform import Rotation as R


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def quat2rot(quat):
    """
    Convert quatnion to rotation matrix

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])

    Returns:
        np.ndarray: rotation matrix (shape: [3, 3])

    """
    r = R.from_quat(quat)
    return r.as_dcm()


def quat2euler(quat, axes='xyz'):
    """
    Convert quatnion to euler angles

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {‘X’, ‘Y’, ‘Z’}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {‘x’, ‘y’, ‘z’} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: [3,])
    """
    r = R.from_quat(quat)
    return r.as_euler(axes)


def quat_inverse(quat):
    """
    Return the quaternion inverse

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])

    Returns:
        np.ndarray: inverse quaternion (shape: [4,])
    """
    r = R.from_quat(quat)
    return r.inv().as_quat()


def quat_multiply(quat1, quat2):
    """
    Quaternion mulitplication

    Args:
        quat1 (list or np.ndarray): first quaternion [x,y,z,w] (shape: [4,])
        quat2 (list or np.ndarray): second quaternion [x,y,z,w] (shape: [4,])

    Returns:
        np.ndarray: quat1 * quat2 (shape: [4,])
    """
    r1 = R.from_quat(quat1)
    r2 = R.from_quat(quat2)
    r = r1 * r2
    return r.as_quat()


def euler2rot(euler, axes='xyz'):
    """
    Convert euler angles to rotation matrix

    Args:
        euler (list or np.ndarray): euler angles (shape: [3,])
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {‘X’, ‘Y’, ‘Z’}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {‘x’, ‘y’, ‘z’} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: rotation matrix (shape: [3, 3])
    """
    r = R.from_euler(axes, euler)
    return r.as_dcm()


def euler2quat(euler, axes='xyz'):
    """
    Convert euler angles to quaternion

    Args:
        euler (list or np.ndarray): euler angles (shape: [3,])
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {‘X’, ‘Y’, ‘Z’}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {‘x’, ‘y’, ‘z’} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: quaternion [x,y,z,w] (shape: [4,])
    """
    r = R.from_euler(axes, euler)
    return r.as_quat()


def rot2quat(rot):
    """
    Convert rotation matrix to quaternion

    Args:
        rot (np.ndarray): rotation matrix (shape: [3, 3])

    Returns:
        np.ndarray: quaternion [x,y,z,w] (shape: [4,])
    """
    r = R.from_dcm(rot)
    return r.as_quat()


def rot2euler(rot, axes='xyz'):
    """
    Convert rotation matrix to euler angles

    Args:
        rot (np.ndarray): rotation matrix (shape: [3, 3])
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {‘X’, ‘Y’, ‘Z’}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {‘x’, ‘y’, ‘z’} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: [3,])
    """
    r = R.from_dcm(rot)
    return r.as_euler(axes)


def kdl_array_to_numpy(kdl_data):
    """
    Convert KDL array data into numpy array

    Args:
        kdl_data: data in KDL format

    Returns:
        numpy array
    """
    np_array = np.zeros((kdl_data.rows(), kdl_data.columns()))
    for i in range(kdl_data.rows()):
        for j in range(kdl_data.columns()):
            np_array[i, j] = kdl_data[i, j]
    return np_array


def kdl_frame_to_numpy(frame):
    """
    Convert KDL Frame data into numpy array

    Args:
        frame: data of KDL Frame

    Returns:
        transformation matrix in numpy [4x4]
    """
    p = frame.p
    M = frame.M
    return np.array([[M[0, 0], M[0, 1], M[0, 2], p.x()],
                     [M[1, 0], M[1, 1], M[1, 2], p.y()],
                     [M[2, 0], M[2, 1], M[2, 2], p.z()],
                     [0, 0, 0, 1]])


def joints_to_kdl(joint_values):
    """
    Convert the numpy array into KDL data format

    Args:
        joint_values (np.ndarray): values for the joints

    Returns:

    """
    num_jts = joint_values.size
    kdl_array = kdl.JntArray(num_jts)
    for idx in range(num_jts):
        kdl_array[idx] = joint_values[idx]
    return kdl_array


def print_red(skk):
    """
    print the text in red color

    Args:
        skk (str): text to be printed
    """
    print("\033[91m {}\033[00m".format(skk))


def print_green(skk):
    """
    print the text in green color

    Args:
        skk (str): text to be printed
    """
    print("\033[92m {}\033[00m".format(skk))


def print_yellow(skk):
    """
    print the text in yellow color

    Args:
        skk (str): text to be printed
    """
    print("\033[93m {}\033[00m".format(skk))


def print_blue(skk):
    """
    print the text in blue color

    Args:
        skk (str): text to be printed
    """
    print("\033[94m {}\033[00m".format(skk))


def print_purple(skk):
    """
    print the text in purple color

    Args:
        skk (str): text to be printed
    """
    print("\033[95m {}\033[00m".format(skk))


def print_cyan(skk):
    """
    print the text in cyan color

    Args:
        skk (str): text to be printed
    """
    print("\033[96m {}\033[00m".format(skk))
