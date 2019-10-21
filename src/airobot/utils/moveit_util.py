import sys
import time

import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import conversions


class MoveitScene(object):
    """
    Use this class to create objects that reside in moveit environments
    """

    def __init__(self):
        """
        Constructor of the MoveitObjectHandler class.
        """
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        time.sleep(1)

    def add_static_obj(self, obj_name, pos, ori, size=None,
                       obj_type='box', ref_frame='/base_link',
                       normal=None):
        """
        Add static object to the planning scene

        Args:
            obj_name (str): object name
            pos (list): position
            ori (list): orientation. It can be quaternion ([qx, qy, qz, qw])
                or euler angles ([roll, pitch, yaw])
            size (float or list or tuple): size of the object.
                if the object is a plane, size should be None
                if the object is a box, size can be a float,
                    which means the edge
                    length of a cube. size can also be a list
                     or tuple of length 3,
                    the it specifies the 3 edge lengths of the cuboid
                if the object is a sphere, size is a float,
                which means the radius
            obj_type (str): one of ['sphere', 'box', 'plane']
            ref_frame (str): reference frame on which
                the pos and ori are specified
            normal (list or tuple): only used if the
                object is a plane. It means the
                normal direction of the plane

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
                                     ' should be 3 for box')
            if isinstance(size, list):
                size = tuple(size)
            self.scene.add_box(obj_name, pose_stamped, size)
        else:
            if not isinstance(size, float):
                raise ValueError('Size should a float number for sphere')
            self.scene.add_sphere(obj_name, pose_stamped, radius=size)

    def add_dynamic_obj(self, ref_frame, obj_name, pos, ori, size,
                        touch_links=None):
        """
        Add object to the ref_frame, the object will move with the ref_frame.
        Only box is supported for now.

        Args:
            ref_frame (str): which link are you adding object to
            obj_name (str): object name
            pos (list): position of the object with respect to the ref_frame
            ori (list): orientation of the object with respect to the ref_frame
            size (float or list or tuple): size can be a
                float, which means the edge
                length of a cube. size can also be a list or tuple of length 3,
                the it specifies the 3 edge lengths of the cuboid

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
                                 ' should be 3 for box')
        if isinstance(size, list):
            size = tuple(size)
        if touch_links is None:
            self.scene.attach_box(ref_frame, obj_name, pose_stamped, size)
        else:
            # moveit ignores collisions between box and links in touch_links
            self.scene.attach_box(ref_frame, obj_name,
                                  pose_stamped, size,
                                  touch_links=touch_links)

    def remove_obj(self, obj_name):
        """
        Remove the object from the planning scene

        Args:
            obj_name (str): object name

        """
        self.scene.remove_world_object(obj_name)

    def remove_all_objs(self):
        """
        Remove all the added objects in the planning scene
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
        Get all the static and dynamic objects in the planning scene
        Returns:

        """
        objs = self.scene.get_objects()
        objs_attached = self.scene.get_attached_objects()
        return objs, objs_attached

    def unlink_obj(self, ref_frame, obj_name=None, delete=True):
        """
        Unlink the attached object from ref_frame

        Args:
            ref_frame: the parent link at which the object is attached to
            obj_name: the object name
            delete: If True, the object will be deleted from the scene.
                Otherwise, the object will be unlinked
                from the parent link only,
                but the object is still there in the scene.

        """
        self.scene.remove_attached_object(ref_frame, obj_name)
        if delete:
            self.remove_obj(obj_name)
