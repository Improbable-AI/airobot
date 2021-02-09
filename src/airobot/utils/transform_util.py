from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from airobot.utils import common


class Position:
    """
    Class for representing 3D positions.

    Attributes:
        x (float): x coordinate.
        y (float): y coordinate.
        z (float): z coordinate.
    """
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.


class Orientation:
    """
    Class for representing 3D orientations,
    as a quaternion.

    Attributes:
        x (float): qx value.
        y (float): qy value.
        z (float): qz value.
        w (float): qw value.
    """
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.w = 0.


class Pose:
    """
    Class to represent a 6D pose, using a 3D position and
    3D orientation, represented as a quaternion.

    Attributes:
        position (Position): 3D position.
        orientation (Orientation): 3D orientation (quaternion).
    """
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class Header:
    """
    Class to represent the header that would accompany a
    6D pose, containing a frame name.

    Attributes:
        frame_id (string): Name of coordinate frame.
    """
    def __init__(self):
        self.frame_id = "world"


class PoseStamped():
    """
    Class to represent a 6D pose, expressed in a particular frame.

    Attributes:
        pose (Pose): 6D pose.
        header (Header): Header, with frame information.
    """
    def __init__(self):
        position = Position()
        orientation = Orientation()
        pose = Pose(position, orientation)
        header = Header()
        self.pose = pose
        self.header = header


def pose_stamped2list(msg):
    """
    Function to convert a pose_stamped into a list

    Args:
        msg (PoseStamped): 6D pose.

    Returns:
        list: 6D pose, as list [x, y, z, qx, qy, qz, qw].
    """
    return [float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
            ]

def list2pose_stamped(pose, frame_id="world"):
    """
    Function to convert a list into a pose_stamped

    Args:
        pose (list): 6D pose, as list [x, y, z, qx, qy, qz, qw].
        frame_id (str, optional): Frame this pose is expressed in.

    Returns:
        PoseStamped: 6D pose as a PoseStamped.
    """
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg


def unit_pose():
    """
    Function to create a canonical pose, centered as the origin
    and not rotated about any axis.

    Returns:
        PoseStamped: 6D canonical pose.
    """
    return list2pose_stamped([0, 0, 0, 0, 0, 0, 1])


def matrix_from_pose(pose):
    """
    Function to convert from a PoseStamped to a 
    homogeneous transformation matrix.

    Args:
        pose (PoseStamped): 6D pose, to convert to transformation matrix. 

    Returns:
        np.ndarray: Homogeneous transformation matrix representation of 
            the pose.
    """
    pose_list = pose_stamped2list(pose)
    trans, quat = pose_list[:3], pose_list[3:]
    T = np.eye(4)
    T[:-1, :-1] = common.quat2rot(quat)
    T[0:3, 3] = trans
    return T


def pose_from_matrix(matrix, frame_id="world"):
    """
    Function to convert from a homogeneous transformation matrix to
    a PoseStamped.

    Args:
        matrix (np.ndarray): Homogeneous transformation matrix,
            shape :math:`4x4`.
        frame_id (str, optional): Reference frame of the pose. 

    Returns:
        PoseStamped: 6D pose representation of the transformation matrix.
    """
    quat = common.rot2quat(matrix[:-1, :-1])
    trans = matrix[:-1, -1]
    pose = list(trans) + list(quat)
    pose = list2pose_stamped(pose, frame_id=frame_id)
    return pose


def get_transform(pose_frame_target, pose_frame_source):
    """
    Function to find a transform that transforms pose source to
    pose target. Both poses must be expressed in the same reference frame.

    Args:
        pose_frame_target (PoseStamped): Target 6D pose.
        pose_frame_source (PoseStamped): Source 6D pose.

    Returns:
        PoseStamped: Relative pose, which, if applied to pose_frame_source,
            will transform it to pose_frame_target.
    """
    #both poses must be expressed in same reference frame
    T_target_world = matrix_from_pose(pose_frame_target)
    T_source_world = matrix_from_pose(pose_frame_source)
    T_relative_world = np.matmul(T_target_world, np.linalg.inv(T_source_world))
    pose_relative_world = pose_from_matrix(
        T_relative_world, frame_id=pose_frame_source.header.frame_id)
    return pose_relative_world


def convert_reference_frame(pose_source, pose_frame_target, pose_frame_source, 
                            frame_id="world"):
    """
    Function to represent a pose expressed in a source reference frame, in a
    different target reference frame.

    Args:
        pose_source (PoseStamped): 6D pose, expressed in pose_frame_source reference frame.
        pose_frame_target (PoseStamped): Reference frame to express the returned pose with
            respect to.
        pose_frame_source (PoseStamped): Reference frame that pose_source is expressed in.  
        frame_id (str): Name to add to the PoseStamped, indicating the name of the target
            reference frame.

    Returns:
        PoseStamped: 6D pose expressed in the target frame.
    """
    T_pose_source = matrix_from_pose(pose_source)
    pose_transform_target2source = get_transform(
        pose_frame_source, pose_frame_target)
    T_pose_transform_target2source = matrix_from_pose(
        pose_transform_target2source)
    T_pose_target = np.matmul(T_pose_transform_target2source, T_pose_source)
    pose_target = pose_from_matrix(T_pose_target, frame_id=frame_id)
    return pose_target


def transform_pose(pose_source, pose_transform):
    """
    Function to apply a world frame transformation to a 6D pose.

    Args:
        pose_source (PoseStamped): Original source pose to apply transformation to. 
        pose_transform (PoseStamped): Transformation to apply to source pose. 

    Returns:
        PoseStamped: Transformed 6D pose.
    """
    T_pose_source = matrix_from_pose(pose_source)
    T_transform_source = matrix_from_pose(pose_transform)
    T_pose_final_source = np.matmul(T_transform_source, T_pose_source)
    pose_final_source = pose_from_matrix(
        T_pose_final_source, frame_id=pose_source.header.frame_id)
    return pose_final_source


def transform_body(pose_source_world, pose_transform_target_body):
    """
    Function to apply a body frame transformation to a 6D pose.

    Args:
        pose_source_world (PoseStamped): Original source pose to transform 
        pose_transform_target_body (PoseStamped): Body frame transformation
            to apply to source pose.

    Returns:
        PoseStamped: Transformed 6D pose. 
    """
    #convert source to target frame
    pose_source_body = convert_reference_frame(pose_source_world,
                                               pose_source_world,
                                               unit_pose(),
                                               frame_id="body_frame")
    #perform transformation in body frame
    pose_source_rotated_body = transform_pose(pose_source_body,
                                              pose_transform_target_body)
    # rotate back
    pose_source_rotated_world = convert_reference_frame(pose_source_rotated_body,
                                                        unit_pose(),
                                                        pose_source_world,
                                                        frame_id="yumi_body")
    return pose_source_rotated_world


def interpolate_pose(pose_initial, pose_final, N):
    """
    Function to interpolate between two poses using a combination of
    linear position interpolation and quaternion spherical-linear
    interpolation (SLERP)

    Args:
        pose_initial (PoseStamped): Initial pose
        pose_final (PoseStamped): Final pose
        N (int): Number of intermediate points.

    Returns:
        list: List of poses that interpolates between initial and final pose.
            Each element is PoseStamped. 
    """
    frame_id = pose_initial.header.frame_id
    pose_initial_list = pose_stamped2list(pose_initial)
    pose_final_list = pose_stamped2list(pose_final)
    trans_initial = pose_initial_list[:3]
    quat_initial = pose_initial_list[3:]

    trans_final = pose_final_list[:3]
    quat_final = pose_final_list[3:]

    trans_interp_total = [np.linspace(trans_initial[0], trans_final[0], num=N),
                          np.linspace(trans_initial[1], trans_final[1], num=N),
                          np.linspace(trans_initial[2], trans_final[2], num=N)]
    
    key_rots = R.from_quat([quat_initial, quat_final])
    slerp = Slerp(np.arange(2), key_rots)
    interp_rots = slerp(np.linspace(0, 1, N))
    quat_interp_total = interp_rots.as_quat()    

    pose_interp = []
    for counter in range(N):
        pose_tmp = [
            trans_interp_total[0][counter],
            trans_interp_total[1][counter],
            trans_interp_total[2][counter],
            quat_interp_total[counter][0], 
            quat_interp_total[counter][1],
            quat_interp_total[counter][2],
            quat_interp_total[counter][3],
        ]
        pose_interp.append(list2pose_stamped(pose_tmp, frame_id=frame_id))
    return pose_interp
