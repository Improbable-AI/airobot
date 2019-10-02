from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import PyKDL as kdl
import numpy as np 

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


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
