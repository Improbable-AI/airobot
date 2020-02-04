from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from airobot.utils.common import load_class_from_path


class ARM(object):
    """
    Base class for robots.

    Args:
        cfgs (YACS CfgNode): configurations for the robot
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class. Defaults to None

    Attributes:
        cfgs (YACS CfgNode): configurations for the robot
        eetool (EndEffectorTool): end effector tool
    """

    def __init__(self, cfgs, eetool_cfg=None):
        self.cfgs = cfgs
        if cfgs.HAS_EETOOL:
            if eetool_cfg is None:
                eetool_cfg = {}
            cls_name = cfgs.EETOOL.CLASS
            from airobot.ee_tool import cls_name_to_path as ee_cls_name_to_path
            eetool_calss = load_class_from_path(cls_name,
                                                ee_cls_name_to_path[cls_name])
            self.eetool = eetool_calss(cfgs, **eetool_cfg)

    def go_home(self):
        """
        Move the robot arm to a pre-defined home pose.
        """
        raise NotImplementedError

    def set_jpos(self, position, joint_name=None, *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits
        """
        raise NotImplementedError

    def set_jvel(self, velocity, joint_name=None, *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies).

        Args:
            velocity (float or list or flattened np.ndarray):
                desired joint velocity(ies)
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits
        """
        raise NotImplementedError

    def set_jtorq(self, torque, joint_name=None, *args, **kwargs):
        """
        Apply torque(s) to the joint(s).

        Args:
            torque (float or list or flattened np.ndarray):
                torque value(s) for the joint(s)
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the actuated joints on the arm. Otherwise,
                only the specified joint will be applied with
                the given torque.
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits

        """
        raise NotImplementedError

    def set_ee_pose(self, pos=None, ori=None, *args, **kwargs):
        """
        Move the end effector to the specifed pose.

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`)
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits
        """
        raise NotImplementedError

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation.

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z directions
            eef_step (float): interpolation interval along delta_xyz.
                Interpolate a point every eef_step distance
                between the two end points

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits
        """
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s).

        Args:
            joint_name (str): If it's None, it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            One of the following

            - float: joint position given joint_name
            - list: joint positions if joint_name is None
        """
        raise NotImplementedError

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies).

        Args:
            joint_name (str): If it's None, it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint velocity of the specified joint

        Returns:
            One of the following

            - float: joint velocity given joint_name
            - list: joint velocities if joint_name is None
        """
        raise NotImplementedError

    def get_jtorq(self, joint_name=None):
        """
        Return the joint torque(s).

        Args:
            joint_name (str): If it's None, it will return joint torques
                of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint

        Returns:
            One of the following

            - float: joint torque given joint_name
            - list: joint torques if joint_name is None
        """
        raise NotImplementedError

    def get_ee_pose(self):
        """
        Return the end effector pose.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3]`)
            - np.ndarray: quaternion representation ([x, y, z, w]) of the EE
              orientation (shape: :math:`[4]`)
            - np.ndarray: rotation matrix representation of the EE orientation
              (shape: :math:`[3, 3]`)
            - np.ndarray: euler angle representation of
              the EE orientation (roll,
              pitch, yaw with static reference frame) (shape: :math:`[3]`)
        """
        raise NotImplementedError

    def compute_ik(self, pos, ori=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector.

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`)
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation

        Returns:
            list: inverse kinematics solution (joint angles)
        """
        raise NotImplementedError
