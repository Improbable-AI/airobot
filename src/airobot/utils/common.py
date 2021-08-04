from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import ast
import glob
import os
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


def ang_in_mpi_ppi(angle):
    """
    Convert the angle to the range [-pi, pi).

    Args:
        angle (float): angle in radians.

    Returns:
        float: equivalent angle in [-pi, pi).
    """

    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle


def clamp(n, minn, maxn):
    """
    Clamp the input value to be in [minn, maxn].

    Args:
        n (float or int): input value.
        minn (float or int): minimum value.
        maxn (float or int): maximum value.

    Returns:
        float or int: clamped value.
    """
    return max(min(maxn, n), minn)


def quat2rot(quat):
    """
    Convert quaternion to rotation matrix.

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: :math:`[4,]`).

    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).

    """
    r = R.from_quat(quat)
    if hasattr(r, 'as_matrix'):
        return r.as_matrix()
    else:
        return r.as_dcm()


def quat2euler(quat, axes='xyz'):
    """
    Convert quaternion to euler angles.

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: :math:`[4,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    r = R.from_quat(quat)
    return r.as_euler(axes)


def quat2rotvec(quat):
    """
    Convert quaternion to rotation vector.

    Arguments:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: :math:`[4,]`).

    Returns:
        np.ndarray: rotation vector (shape: :math:`[3,]`).
    """
    r = R.from_quat(quat)
    return r.as_rotvec()


def quat_inverse(quat):
    """
    Return the quaternion inverse.

    Args:
        quat (list or np.ndarray): quaternion [x,y,z,w] (shape: :math:`[4,]`).

    Returns:
        np.ndarray: inverse quaternion (shape: :math:`[4,]`).
    """
    r = R.from_quat(quat)
    return r.inv().as_quat()


def quat_multiply(quat1, quat2):
    """
    Quaternion mulitplication.

    Args:
        quat1 (list or np.ndarray): first quaternion [x,y,z,w]
            (shape: :math:`[4,]`).
        quat2 (list or np.ndarray): second quaternion [x,y,z,w]
            (shape: :math:`[4,]`).

    Returns:
        np.ndarray: quat1 * quat2 (shape: :math:`[4,]`).
    """
    r1 = R.from_quat(quat1)
    r2 = R.from_quat(quat2)
    r = r1 * r2
    return r.as_quat()


def rotvec2rot(vec):
    """
    A rotation vector is a 3 dimensional vector which is
    co-directional to the axis of rotation and whose
    norm gives the angle of rotation (in radians).

    Args:
        vec (list or np.ndarray): a rotational vector. Its norm
            represents the angle of rotation.

    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).
    """
    r = R.from_rotvec(vec)
    if hasattr(r, 'as_matrix'):
        return r.as_matrix()
    else:
        return r.as_dcm()


def rotvec2quat(vec):
    """
    A rotation vector is a 3 dimensional vector which is
    co-directional to the axis of rotation and whose
    norm gives the angle of rotation (in radians).

    Args:
        vec (list or np.ndarray): a rotational vector. Its norm
            represents the angle of rotation.

    Returns:
        np.ndarray: quaternion [x,y,z,w] (shape: :math:`[4,]`).
    """
    r = R.from_rotvec(vec)
    return r.as_quat()


def rotvec2euler(vec, axes='xyz'):
    """
    A rotation vector is a 3 dimensional vector which is
    co-directional to the axis of rotation and whose
    norm gives the angle of rotation (in radians).

    Args:
        vec (list or np.ndarray): a rotational vector. Its norm
            represents the angle of rotation.
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    r = R.from_rotvec(vec)
    return r.as_euler(axes)


def euler2rot(euler, axes='xyz'):
    """
    Convert euler angles to rotation matrix.

    Args:
        euler (list or np.ndarray): euler angles (shape: :math:`[3,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: rotation matrix (shape: :math:`[3, 3]`).
    """
    r = R.from_euler(axes, euler)
    if hasattr(r, 'as_matrix'):
        return r.as_matrix()
    else:
        return r.as_dcm()


def euler2quat(euler, axes='xyz'):
    """
    Convert euler angles to quaternion.

    Args:
        euler (list or np.ndarray): euler angles (shape: :math:`[3,]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: quaternion [x,y,z,w] (shape: :math:`[4,]`).
    """
    r = R.from_euler(axes, euler)
    return r.as_quat()


def rot2quat(rot):
    """
    Convert rotation matrix to quaternion.

    Args:
        rot (np.ndarray): rotation matrix (shape: :math:`[3, 3]`).

    Returns:
        np.ndarray: quaternion [x,y,z,w] (shape: :math:`[4,]`).
    """
    if hasattr(R, 'from_matrix'):
        r = R.from_matrix(rot)
    else:
        r = R.from_dcm(rot)
    return r.as_quat()


def rot2rotvec(rot):
    """
    Convert rotation matrix to quaternion.

    Args:
        rot (np.ndarray): rotation matrix (shape: :math:`[3, 3]`).

    Returns:
        np.ndarray: a rotation vector (shape: :math:`[3,]`).
    """
    if hasattr(R, 'from_matrix'):
        r = R.from_matrix(rot)
    else:
        r = R.from_dcm(rot)
    return r.as_rotvec()


def rot2euler(rot, axes='xyz'):
    """
    Convert rotation matrix to euler angles.

    Args:
        rot (np.ndarray): rotation matrix (shape: :math:`[3, 3]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
    """
    if hasattr(R, 'from_matrix'):
        r = R.from_matrix(rot)
    else:
        r = R.from_dcm(rot)
    return r.as_euler(axes)


def to_rot_mat(ori):
    """
    Convert orientation in any form (rotation matrix,
    quaternion, or euler angles) to rotation matrix.

    Args:
        ori (list or np.ndarray): orientation in any following form:
            rotation matrix (shape: :math:`[3, 3]`)
            quaternion (shape: :math:`[4]`)
            euler angles (shape: :math:`[3]`).

    Returns:
        np.ndarray: orientation matrix (shape: :math:`[3, 3]`).
    """

    ori = np.array(ori)
    if ori.size == 3:
        # [roll, pitch, yaw]
        ori = euler2rot(ori)
    elif ori.size == 4:
        ori = quat2rot(ori)
    elif ori.shape != (3, 3):
        raise ValueError('Orientation should be rotation matrix, '
                         'euler angles or quaternion')
    return ori


def to_euler_angles(ori, axes='xyz'):
    """
    Convert orientation in any form (rotation matrix,
    quaternion, or euler angles) to euler angles (roll, pitch, yaw).

    Args:
        ori (list or np.ndarray): orientation in any following form:
            rotation matrix (shape: :math:`[3, 3]`)
            quaternion (shape: :math:`[4]`)
            euler angles (shape: :math:`[3]`).
        axes (str): Specifies sequence of axes for rotations.
            3 characters belonging to the set {'X', 'Y', 'Z'}
            for intrinsic rotations (rotation about the axes of a
            coordinate system XYZ attached to a moving body),
            or {'x', 'y', 'z'} for extrinsic rotations (rotation about
            the axes of the fixed coordinate system).

    Returns:
        np.ndarray: euler angles (shape: :math:`[3,]`).
            By default, it's [roll, pitch, yaw]

    """
    ori = np.array(ori)
    if ori.size == 4:
        ori = quat2euler(ori, axes=axes)
    elif ori.shape == (3, 3):
        ori = rot2euler(ori, axes=axes)
    elif ori.size != 3:
        raise ValueError('Orientation should be rotation matrix, '
                         'euler angles or quaternion')
    return ori


def to_quat(ori):
    """
    Convert orientation in any form (rotation matrix,
    quaternion, or euler angles) to quaternion.

    Args:
        ori (list or np.ndarray): orientation in any following form:
            rotation matrix (shape: :math:`[3, 3]`)
            quaternion (shape: :math:`[4]`)
            euler angles (shape: :math:`[3]`).

    Returns:
        np.ndarray: quaternion [x, y, z, w](shape: :math:`[4, ]`).
    """
    ori = np.array(ori)
    if ori.size == 3:
        # [roll, pitch, yaw]
        ori = euler2quat(ori)
    elif ori.shape == (3, 3):
        ori = rot2quat(ori)
    elif ori.size != 4:
        raise ValueError('Orientation should be rotation matrix, '
                         'euler angles or quaternion')
    return ori


def se3_to_trans_ori(se3, ori='quat', axes='xyz'):
    """

    Args:
        se3 (np.ndarray): a SE(3) matrix (shape: :math:`[4, 4]`)
        ori (str): orientation format, can be one of ['quat', 'euler', 'matrix', 'rotvec']
        axes (str): only used when ori == 'euler'

    Returns:
        2-element tuple containing

        - np.ndarray: translational vector (shape: :math:`[3,]`).
        - np.ndarray: rotational vector/matrix.
    """
    rot = se3[:3, :3]
    trans = se3[:3, 3]
    if ori == 'quat':
        ori = to_quat(rot)
    elif ori == 'euler':
        ori = to_euler_angles(rot, axes=axes)
    elif ori == 'matrix':
        ori = rot
    elif ori == 'rotvec':
        ori = rot2rotvec(rot)
    else:
        raise ValueError('Unknown orientation format:{ori}.'.format(ori=ori))
    return trans, ori


def create_se3(ori, trans=None):
    """
    Args:
        ori (np.ndarray): orientation in any following form:
            rotation matrix (shape: :math:`[3, 3]`)
            quaternion (shape: :math:`[4]`)
            euler angles (shape: :math:`[3]`).
        trans (np.ndarray): translational vector (shape: :math:`[3]`)

    Returns:
        np.ndarray: a transformation matrix (shape: :math:`[4, 4]`)
    """

    rot = to_rot_mat(ori)
    out = np.eye(4)
    out[:3, :3] = rot
    if trans is not None:
        trans = np.array(trans)
        out[:3, 3] = trans.flatten()
    return out


def print_red(skk):
    """
    print the text in red color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[91m {}\033[00m".format(skk))


def print_green(skk):
    """
    print the text in green color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[92m {}\033[00m".format(skk))


def print_yellow(skk):
    """
    print the text in yellow color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[93m {}\033[00m".format(skk))


def print_blue(skk):
    """
    print the text in blue color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[94m {}\033[00m".format(skk))


def print_purple(skk):
    """
    print the text in purple color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[95m {}\033[00m".format(skk))


def print_cyan(skk):
    """
    print the text in cyan color.

    Args:
        skk (str): text to be printed.
    """
    print("\033[96m {}\033[00m".format(skk))


def create_folder(path, delete=True):
    """
    Create a new folder.

    Args:
        path (str): path of the folder.
        delete (bool): if delete=True, then if the path already
            exists, the folder will be deleted and recreated.
    """
    if delete and os.path.exists(path):
        shutil.rmtree(path)
    if not os.path.exists(path):
        os.makedirs(path)


def list_class_names(dir_path):
    """
    Return the mapping of class names in all files
    in dir_path to their file path.

    Args:
        dir_path (str): absolute path of the folder.

    Returns:
        dict: mapping from the class names in all python files in the
        folder to their file path.

    """

    py_files = glob.glob(os.path.join(dir_path, "*.py"))
    py_files = [f for f in py_files if os.path.isfile(f) and
                not f.endswith('__init__.py')]
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
    """
    Load a class from the file path.

    Args:
        cls_name (str): class name.
        path (str): python file path.

    Returns:
        Python Class: return the class A which is named as cls_name.
        You can call A() to create an instance of this class using
        the return value.

    """
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


def linear_interpolate_path(start_pos, delta_xyz, interval):
    """
    Linear interpolation in a path.

    Args:
        start_pos (list or np.ndarray): start position
            ([x, y, z], shape: :math:`[3]`).
        delta_xyz (list or np.ndarray): movement in x, y, z
            directions (shape: :math:`[3,]`).
        interval (float): interpolation interval along delta_xyz.
            Interpolate a point every `interval` distance
            between the two end points.

    Returns:
        np.ndarray: waypoints along the path (shape: :math:`[N, 3]`).

    """
    start_pos = np.array(start_pos).flatten()
    delta_xyz = np.array(delta_xyz).flatten()
    path_len = np.linalg.norm(delta_xyz)
    num_pts = int(np.ceil(path_len / float(interval)))
    if num_pts <= 1:
        num_pts = 2
    waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
    waypoints = start_pos + waypoints_sp / float(path_len) * delta_xyz
    return waypoints
