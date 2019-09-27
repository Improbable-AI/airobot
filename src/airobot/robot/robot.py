from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class Robot(object):
    """
    Base class for robots
    """

    def __init__(self, cfgs):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the robot
        """
        self.cfgs = cfgs
        self.camera = None

    def go_home(self):
        """
        Move the robot arm to a pre-defined home pose
        """
        raise NotImplementedError

    def set_jpos(self, position, joint_name=None, *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def set_jvel(self, velocity, joint_name=None, *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies).

        Args:
            velocity (float or list): desired joint velocity(ies)
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def set_jtorq(self, torque, joint_name=None, *args, **kwargs):
        """
        Apply torque(s) to the joint(s)

        Args:
            torque (float or list): torque value(s) for the joint(s)
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the actuated joints on the arm. Otherwise,
                only the specified joint will be applied with
                the given torque.
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits

        """
        raise NotImplementedError

    def set_ee_pose(self, pos, ori=None, *args, **kwargs):
        """
        Move the end effector to the specifed pose
        Args:
            pos (list): position
            ori (list): orientation. It can be either quaternion (length is 4)
                or euler angles ([roll, pitch, yaw])

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation

        Args:
            delta_xyz: movement in x, y, z directions
            eef_step: interpolation interval along delta_xyz. Interpolate
                a point every eef_step distance between the two end points

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s)

        Args:
            joint_name: If it's None, it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            joint position (float) or joint positions (list) depends on
            the joint_name
        """
        raise NotImplementedError

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies)

        Args:
            joint_name: If it's None, it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint velocity of the specified joint

        Returns:
            joint velocity (float) or joint velocities (list) depends on
            the joint_name
        """
        raise NotImplementedError

    def get_jtorq(self, joint_name=None):
        """
        Return the joint torque(s)

        Args:
            joint_name: If it's None, it will return joint torques
                of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint

        Returns:
            joint torque (float) or joint torques (list) depends on
            the joint_name
        """
        raise NotImplementedError

    def get_ee_pose(self):
        """
        Return the end effector pose

        Returns:
            end effector position, quaternion,
            rotation matrix, euler angles
        """
        raise NotImplementedError

    def compute_ik(self, pos, ori=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector

        Args:
            pos (list): position
            ori (list): orientation. It can be euler angles
                (roll, pitch, yaw) or quaternion.

        Returns:
            inverse kinematics solution (joint angles, list)
        """
        raise NotImplementedError
