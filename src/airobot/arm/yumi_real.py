"""
Interface to ABB Yumi robot.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import airobot.utils.common as arutil
from airobot.arm.dual_arm_real import DualArmReal
from airobot.arm.yumi_arm_real import YumiArmReal


class YumiReal(DualArmReal):
    """
    Class interfacing with real ABB Yumi robot though ROS

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        moveit_planner (str): motion planning algorithm.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.
        wrist_cam (bool): whether the robot has a wrist camera
            mounted. If so, a box will be placed around the camera
            so that moveit is aware of the wrist camera when it's
            doing motion planning.

    Attributes:
        right_arm (YumiArmReal): Right arm instance
        left_arm (YumiArmReal): Left arm instance
    """

    def __init__(self, cfgs,
                 moveit_planner='RRTstarkConfigDefault',
                 eetool_cfg=None,
                 wrist_cam=True):
        super(YumiReal, self).__init__(cfgs=cfgs,
                                       eetool_cfg=eetool_cfg)
        right_cfg = cfgs.ARM.RIGHT
        left_cfg = cfgs.ARM.LEFT
        self.right_arm = YumiArmReal(cfgs=right_cfg,
                                     moveit_planner=moveit_planner,
                                     eetool_cfg=eetool_cfg,
                                     wrist_cam=wrist_cam)
        self.left_arm = YumiArmReal(cfgs=left_cfg,
                                    moveit_planner=moveit_planner,
                                    eetool_cfg=eetool_cfg,
                                    wrist_cam=wrist_cam)

        self.setup_single_arms(right_arm=self.right_arm,
                               left_arm=self.left_arm)

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
        success = False
        for key in self.arms.keys():
            if not self.arms[key].egm_is_active():
                self.arms[key].start_egm()
        if arm is None:
            r_pos = position[:self.single_arm_dof]
            l_pos = position[self.single_arm_dof:]
            pos_to_send = [r_pos, l_pos]
            for i, arm in enumerate(self.arms.values()):
                arm.set_jpos(pos_to_send[i], wait=wait)
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].set_jpos(position,
                                              joint_name=joint_name,
                                              wait=wait)
        return success

    def get_jpos(self, joint_name=None, *args, **kwargs):
        both_arm_jnt_names = self.right_arm.arm_jnt_names + \
                             self.left_arm.arm_jnt_names

        r_arm_pos = self.right_arm.get_jpos()
        l_arm_pos = self.left_arm.get_jpos()
        both_arm_pos = r_arm_pos + l_arm_pos
        if joint_name is None:
            jpos = both_arm_pos
        else:
            if joint_name not in both_arm_jnt_names:
                raise ValueError('Joint name [%s] '
                                 'not recognized!' % joint_name)
            jnt_id = both_arm_jnt_names.index(joint_name)
            jpos = both_arm_pos[jnt_id]
        return jpos

    def get_ee_pose(self, arm=None):
        """
        Return the end effector pose.

        Args:
            arm (str): Returned pose will be for specified arm, must
                match arm names in cfg file.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3,]`).
            - np.ndarray: quaternion representation of the
              EE orientation (shape: :math:`[4,]`).
            - np.ndarray: rotation matrix representation of the
              EE orientation (shape: :math:`[3, 3]`).
            - np.ndarray: euler angle representation of the
              EE orientation (roll, pitch, yaw with
              static reference frame) (shape: :math:`[3,]`).
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))    
        return self.arms[arm].get_ee_pose()
