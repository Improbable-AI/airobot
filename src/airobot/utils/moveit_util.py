import copy
import sys
import time

import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_commander import conversions


class MoveitScene(object):
    """
    Use this class to create objects that reside in moveit environments.

    Attributes:
        scene (moveit_commander.PlanningSceneInterface): interface
            to the MoveIt! planning scene.
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        time.sleep(1)

    def add_static_obj(self, obj_name, pos, ori, size=None,
                       obj_type='box', ref_frame='/base_link',
                       normal=None):
        """
        Add static object to the planning scene.

        Args:
            obj_name (str): object name.
            pos (list): position.
            ori (list): orientation. It can be quaternion ([qx, qy, qz, qw])
                or euler angles ([roll, pitch, yaw]).
            size (float or list or tuple): size of the object.
                if the object is a plane, size should be None
                if the object is a box, size can be a float
                (which means the edge length of a cube.)
                size can also be a list or tuple of length 3,
                that specifies the 3 edge lengths of the cuboid
                if the object is a sphere, size is a float,
                (which means the radius).
            obj_type (str): one of [`sphere`, `box`, `plane`].
            ref_frame (str): reference frame on which
                the pos and ori are specified.
            normal (list or tuple): only used if the
                object is a plane. It means the
                normal direction of the plane.

        Returns:
            bool: if the object is successfully added.

        """
        if not isinstance(pos, list):
            raise TypeError('pos should be a list')
        if not isinstance(ori, list):
            raise TypeError('ori should be a list')
        if obj_type not in ['sphere', 'box', 'plane']:
            raise ValueError('Unsupported object type. Only [sphere],'
                             ' [box], [plane] are supported for now.')

        pose = pos + ori
        pose = conversions.list_to_pose(pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = ref_frame
        pose_stamped.pose = pose

        if obj_type == 'plane':
            if size is not None:
                raise ValueError('No size needed for plane')
            if normal is None:
                normal = (0, 0, 1)
            if len(normal) != 3:
                raise ValueError('Length of the normal should be 3')
            if isinstance(normal, list):
                normal = tuple(normal)
            self.scene.add_plane(obj_name, pose_stamped, normal)
        elif obj_type == 'box':
            if isinstance(size, float):
                size = (size, size, size)
            elif isinstance(size, list) or isinstance(size, tuple):
                if len(size) != 3:
                    raise ValueError('If size is a list or tuple, its length'
                                     ' should be 3 for a box')
            else:
                raise TypeError('size should be a float number, '
                                'a 3-element list '
                                'or a 3-element tuple for a box')
            if isinstance(size, list):
                size = tuple(size)
            self.scene.add_box(obj_name, pose_stamped, size)
        else:
            if not isinstance(size, float):
                raise ValueError('Size should a float number for sphere')
            self.scene.add_sphere(obj_name, pose_stamped, radius=size)

        obj_dict, obj_adict = self.get_objects()
        success = False
        if obj_name in obj_dict.keys():
            success = True
        return success

    def add_dynamic_obj(self, ref_frame, obj_name, pos, ori, size,
                        touch_links=None):
        """
        Add object to the ref_frame, the object will move with the ref_frame.
        Only box is supported for now.

        Args:
            ref_frame (str): which link are you adding object to.
            obj_name (str): object name.
            pos (list): position of the object with respect to the ref_frame.
            ori (list): orientation of the object with
                respect to the ref_frame.
            size (float or list or tuple): size can be a
                float, which means the edge
                length of a cube. size can also be a list or tuple of length 3,
                the it specifies the 3 edge lengths of the cuboid.

        Returns:
            bool: if the object is successfully added.

        """
        if not isinstance(pos, list):
            raise TypeError('pos should be a list')
        if not isinstance(ori, list):
            raise TypeError('ori should be a list')

        pose = pos + ori
        pose = conversions.list_to_pose(pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = ref_frame
        pose_stamped.pose = pose

        if isinstance(size, float):
            size = (size, size, size)
        elif isinstance(size, list) or isinstance(size, tuple):
            if len(size) != 3:
                raise ValueError('If size is a list or tuple, its length'
                                 ' should be 3 for a box')
        else:
            raise TypeError('size should be a float number, a 3-element list '
                            'or a 3-element tuple for a box')
        if isinstance(size, list):
            size = tuple(size)
        if touch_links is None:
            self.scene.attach_box(ref_frame, obj_name, pose_stamped, size)
        else:
            # moveit ignores collisions between box and links in touch_links
            self.scene.attach_box(ref_frame, obj_name,
                                  pose_stamped, size,
                                  touch_links=touch_links)
        obj_dict, obj_adict = self.get_objects()
        success = False
        if obj_name in obj_adict.keys():
            success = True
        return success

    def remove_obj(self, obj_name):
        """
        Remove the object from the planning scene.

        Args:
            obj_name (str): object name.

        """
        self.scene.remove_world_object(obj_name)

    def remove_all_objs(self):
        """
        Remove all the added objects in the planning scene.
        """
        objs = self.scene.get_objects()
        objs_attached = self.scene.get_attached_objects()
        # remove add objects
        for key in objs.keys():
            self.remove_obj(key)
        # remove attached objects
        for key in objs_attached.keys():
            self.unlink_obj(objs_attached[key].link_name, key)

    def get_objects(self):
        """
        Get all the static and dynamic objects in the planning scene.

        Returns:
            2-element tuple containing

            - list: static objects.
            - list: dynamic objects.
        """
        objs = self.scene.get_objects()
        objs_attached = self.scene.get_attached_objects()
        return objs, objs_attached

    def unlink_obj(self, ref_frame, obj_name=None, delete=True):
        """
        Unlink the attached object from ref_frame.

        Args:
            ref_frame (str): the parent link at which the
                object is attached to.
            obj_name (str): the object name.
            delete (bool): If True, the object will be deleted from the scene.
                Otherwise, the object will be unlinked
                from the parent link only,
                but the object is still there in the scene.

        """
        self.scene.remove_attached_object(ref_frame, obj_name)
        if delete:
            self.remove_obj(obj_name)


def moveit_cartesian_path(start_pos, start_quat,
                          delta_xyz, moveit_group,
                          eef_step, jump_threshold=0.0):
    """
    Compute the motion plan for cartesian path.

    Args:
        start_pos (list or np.ndarray): start position (shape: :math:`[3]`).
        start_quat (list or np.ndarray): start quaternion
            [x, y, z, w] (shape: :math:`[4]`).
        delta_xyz (list or np.ndarray): Goal change in x, y, z position of
            end effector.
        moveit_group (MoveGroupCommander): moveit group commander.
        eef_step (float): Discretization step in cartesian space
            for computing waypoints along the path.
        jump_threshold (float): Value indicating the maximum allowable joint
            space jump between time steps. Small values are more conservative,
            not allowing large rapid joint movements. A value of 0.0 tells
            MoveIt! to ignore this parameter, allowing for infinitely large
            configuration space jumps.

    Returns:
        moveit_msgs/RobotTrajectory: motion plan to move the end
        effector in a straight line.

    """
    start_pos = np.array(start_pos).flatten()

    delta_xyz = np.array(delta_xyz).flatten()
    end_pos = start_pos + delta_xyz
    moveit_waypoints = []
    wpose = moveit_group.get_current_pose().pose
    wpose.position.x = start_pos[0]
    wpose.position.y = start_pos[1]
    wpose.position.z = start_pos[2]
    wpose.orientation.x = start_quat[0]
    wpose.orientation.y = start_quat[1]
    wpose.orientation.z = start_quat[2]
    wpose.orientation.w = start_quat[3]
    moveit_waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = end_pos[0]
    wpose.position.y = end_pos[1]
    wpose.position.z = end_pos[2]
    wpose.orientation.x = start_quat[0]
    wpose.orientation.y = start_quat[1]
    wpose.orientation.z = start_quat[2]
    wpose.orientation.w = start_quat[3]
    moveit_waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = moveit_group.compute_cartesian_path(
        moveit_waypoints,  # waypoints to follow
        eef_step,  # eef_step
        jump_threshold)  # jump_threshold
    return plan
