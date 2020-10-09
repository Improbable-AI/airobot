from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy

from airobot.arm.arm import ARM
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python import trac_ik
from gym.utils import seeding


class DualArmReal(ARM):
    """Base class for a dual arm robot

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        seed (int): random seed.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.

    Attributes:
        cfgs (YACS CfgNode): configurations for the robot.
        robot_id (int): pybullet body unique id of the robot.
        arms (dict): internal dictionary keyed by the names of each
            single arm, with values as interfaces to the arms.
        arm_jnt_names (list): names of the arm joints.
        right_arm_jnt_names (list): names of the arm joints on the right arm.
        left_arm_jnt_names (list): names of the arm joints on the left arm.
        arm_jnt_ids (list): pybullet joint ids of the arm joints.
        r_ee_link_jnt (str): name of the end effector link on the right arm.
        l_ee_link_jnt (str): name of the end effector link on the left arm.
        dual_arm_dof (int): total number of arm joints in the robot.
        single_arm_dof (int): number of joints in a single arm of the robot.
        jnt_to_id (dict): dictionary with [joint name : pybullet joint] id
            [key : value] pairs.
        non_fixed_jnt_names (list): names of non-fixed joints in the arms,
            used for returning the correct inverse kinematics solution.
    """

    def __init__(self, cfgs, seed=None, eetool_cfg=None):
        super(DualArmReal, self).__init__(cfgs=cfgs, eetool_cfg=eetool_cfg)

        self._np_random, _ = self._seed(seed)

        self.arms = {}
        self._init_real_consts()

    @property
    def _arm_names(self):
        return self.arms.keys()

    def setup_single_arms(self, right_arm, left_arm):
        """
        Function to setup the single arm instances, and
        maintain an internal dictionary interface to them.

        Args:
            right_arm (SingleArmReal): Right arm instance.
            left_arm (SingleArmReal): Left arm instance.
        """
        self._r_arm_name = self.cfgs.ARM.RIGHT.ARM.NAME
        self._l_arm_name = self.cfgs.ARM.LEFT.ARM.NAME
        self.arms[self._r_arm_name] = right_arm
        self.arms[self._l_arm_name] = left_arm

    def go_home(self, arm=None):
        """
        Move the robot to a pre-defined home pose

        Args:
            arm (str): Which arm to move to home. Defaults to None.
        """
        # if arm is None:
        #     success = self.set_jpos(self._home_position)
        # else:
        #     if arm not in self.arms:
        #         raise ValueError('Valid arm name must be specified, '
        #                          '("%s" or "%s")'
        #                          % (self._arm_names[0], self._arm_names[1]))
        #     success = self.arms[arm].go_home()
        # return success
        print('Not implemented! Please plan collision free path to home and uset set_jpos() to execute the motion')
        pass

    def set_jpos(self, position, arm=None, joint_name=None,
                 wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad).

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[6,]` if list, otherwise a single value).
            arm (str): If not provided, position should be a list and all
                actuated joints will be moved. If provided, only have the
                joints will move, corresponding to which arm was specified.
                Value should match arm names in cfg file.
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None.
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        # raise NotImplementedError
        # TODO: check if EGM is already activated
        for arm in self.arms:
            arm.start_egm()
        for i, arm in enumerate(self.arms):
            arm.set_jpos(position[i])

    def set_jvel(self, velocity, arm=None, joint_name=None,
                 wait=False, *args, **kwargs):
        """
        Set joint velocity command to the robot (units in rad/s).

        Args:
            velocity (float or list or flattened np.ndarray): list of target
                joint velocity value(s)
                (shape: :math:`[DOF,]` if list, otherwise a single value).
            arm (str): If not provided, velocity should be a list and all
                actuated joints will be moved. If provided, only have the
                joints will move, corresponding to which arm was specified.
                Value should match arm names in cfg file.                
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): If True, block until robot reaches
                desired joint velocity value(s). Defaults to False.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        raise NotImplementedError

    def _seed(self, seed=None):
        np_random, seed = seeding.np_random(seed)
        return np_random, seed

    def compute_ik(self, pos, ori=None, arm=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector.

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`)
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[3,]`), or
                quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`).
            arm (str): Which arm EE pose corresponds to, must
                match arm names in cfg file

        Returns:
            list: solution to inverse kinematics, joint angles which achieve
            the specified EE pose (shape: :math:`[DOF]`).
        """
        # if arm is None:
        #     raise NotImplementedError
        # else:
        #     if arm not in self.arms:
        #         raise ValueError('Valid arm name must be specified '
        #                          '("%s" or "%s")'
        #                          % (self._arm_names[0], self._arm_names[1]))
        #     return self.arms[arm].compute_ik(pos=pos, ori=ori, ns=ns)
        raise NotImplementedError

    def _check_arm(self, joint_name):
        """Checks which arm a joint is part of
        
        Args:
            joint_name (str): Name of the joint to check

        Returns:
            str: Name of the arm that has the specified joint
        """
        if joint_name in self.right_arm_jnt_names:
            arm_name = self._r_arm_name
        elif joint_name in self.left_arm_jnt_names:
            arm_name = self._l_arm_name
        else:
            raise ValueError('Joint name not recognized')
        return arm_name

    def _init_real_consts(self):
        """
        Initialize constants
        """
        self._home_position = self.cfgs.ARM.HOME_POSITION

        self.right_arm_jnt_names = self.cfgs.ARM.RIGHT.ARM.JOINT_NAMES
        self.left_arm_jnt_names = self.cfgs.ARM.LEFT.ARM.JOINT_NAMES

        self.arm_jnt_names = self.right_arm_jnt_names + self.left_arm_jnt_names

        self.dual_arm_dof = len(self.arm_jnt_names)
        self.single_arm_dof = int(self.dual_arm_dof / 2)
