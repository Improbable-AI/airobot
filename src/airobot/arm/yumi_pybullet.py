"""
Pybullet simulation environment of an ABB Yumi
robot
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pybullet as p

import airobot.utils.common as arutil
from airobot.arm.dual_arm_pybullet import DualArmPybullet
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.utils.pb_util import PB_CLIENT


class YumiPybullet(DualArmPybullet):
    """
    Class for pybullet simulation of ABB Yumi robot with
    separate functionality for both arms
    """

    def __init__(self, cfgs, render=False, seed=None, 
                 rt_simulation=True, self_collision=False,
                 eetool_cfg=None):
        """
        Constructor

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            render (bool): whether to render the environment using GUI
            seed (int): random seed
            rt_simulation (bool): turn on realtime simulation or not
            self_collision (bool): enable self_collision or
                                   not whiling loading URDF
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
        """
        super(YumiPybullet, self).__init__(cfgs=cfgs,
                                           render=render,
                                           seed=seed,
                                           self_collision=self_collision,
                                           eetool_cfg=eetool_cfg,
                                           rt_simulation=rt_simulation)
        right_cfg = cfgs.ARM.RIGHT
        left_cfg = cfgs.ARM.LEFT
        self.right_arm = SingleArmPybullet(cfgs=right_cfg,
                                           render=render,
                                           seed=seed,
                                           self_collision=self_collision,
                                           eetool_cfg=eetool_cfg,
                                           rt_simulation=rt_simulation)
        self.left_arm = SingleArmPybullet(cfgs=left_cfg,
                                          render=render,
                                          seed=seed,
                                          self_collision=self_collision,
                                          eetool_cfg=eetool_cfg,
                                          rt_simulation=rt_simulation)
        self.reset()

    def reset(self):
        """
        Reset the simulation environment.
        """
        p.resetSimulation(physicsClientId=PB_CLIENT)

        yumi_pos = self.cfgs.ARM.PYBULLET_RESET_POS
        yumi_ori = arutil.euler2quat(self.cfgs.ARM.PYBULLET_RESET_ORI)
        if self.self_collision:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       yumi_pos,
                                       yumi_ori,
                                       flags=p.URDF_USE_SELF_COLLISION,
                                       physicsClientId=PB_CLIENT)
        else:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       yumi_pos, yumi_ori,
                                       physicsClientId=PB_CLIENT)

        self._build_jnt_id()

        self.setup_single_arms(right_arm=self.right_arm,
                               left_arm=self.left_arm)

        for arm in self.arm_dict.values():
            if hasattr(arm, 'eetool'):
                arm.eetool.feed_robot_info(self.robot_id, self.jnt_to_id)
                arm.eetool.activate()
                if arm.self_collision:
                    arm.eetool.disable_gripper_self_collision()
