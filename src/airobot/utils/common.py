from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import shutil
import glob
import ast
import sys


import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def quat2rot(quat):
    """
    Convert quaternion to rotation matrix

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])

    Returns:
        np.ndarray: rotation matrix (shape: [3, 3])

    """
    r = R.from_quat(quat)
    return r.as_dcm()


def quat2euler(quat, axes='xyz'):
    """
    Convert quaternion to euler angles

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: [3,])
    """
    r = R.from_quat(quat)
    return r.as_euler(axes)


def quat2rotvec(quat):
    """
    Convert quaternion to rotation vector

    Arguments:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: [4,])

    Returns:
        np.ndarray: rotation vector (shape: [3,])
    """
    r = R.from_quat(quat)
    return r.as_rotvec()


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
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
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
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
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
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
             the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: [3,])
    """
    r = R.from_dcm(rot)
    return r.as_euler(axes)


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


def create_folder(path, delete=True):
    """
    Create a new folder

    Args:
        path (str): path of the folder
        delete (bool): if delete=True, then if the path already
            exists, the folder will be deleted and recreated.
    """
    if delete and os.path.exists(path):
        shutil.rmtree(path)
    if not os.path.exists(path):
        os.makedirs(path)


def list_class_names(dir_path):
    """
    Return the mapping of class names in all files in dir_path to their file path

    Args:
        dir_path (str): absolute path of the folder

    Returns:
        dict: mapping from the class names in all python files in the
            folder to their file path

    """

    py_files = glob.glob(os.path.join(dir_path, "*.py"))
    py_files = [f for f in py_files if os.path.isfile(f)
                and not f.endswith('__init__.py')]
    cls_name_to_path = dict()
    for py_file in py_files:
        with open(py_file) as f:
            node = ast.parse(f.read())
        classes_in_file = [n for n in node.body if isinstance(n, ast.ClassDef)]
        cls_names_in_file = [c.name for c in classes_in_file]
        for cls_name in cls_names_in_file:
            cls_name_to_path[cls_name] = py_file
    return cls_name_to_path


def load_class_from_path(cls_name, path):
    mod_name = 'MOD%s' % cls_name
    if sys.version_info.major == 2:
        import imp
        mod = imp.load_source(mod_name, path)
        return getattr(mod, cls_name)
    elif sys.version_info.major == 3:
        if sys.version_info.minor < 5:
            from importlib.machinery import SourceFileLoader

            mod = SourceFileLoader(mod_name, path).load_module()
            return getattr(mod, cls_name)
        else:
            import importlib.util
            spec = importlib.util.spec_from_file_location(mod_name, path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return getattr(mod, cls_name)
    else:
        raise NotImplementedError



