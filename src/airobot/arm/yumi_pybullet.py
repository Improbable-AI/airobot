"""
Pybullet simulation environment of a UR5e
robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pybullet as p

import airobot.utils.common as arutil
from airobot.utils.pb_util import PB_CLIENT
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.arm.dual_arm_pybullet import DualArmPybullet


class YumiPybullet(DualArmPybullet):
    """
    Class for pybullet simulation of ABB Yumi robot with
    separate functionality for both arms
    """

    def __init__(self, cfgs, render=False, seed=None, self_collision=False,
                 eetool_cfg=None, rt_simulation=True):
        """
        Constructor

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            render (bool): whether to render the environment using GUI
            seed (int): random seed
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
        p.resetSimulation()

        # plane_pos = [0, 0, 0]
        # plane_ori = arutil.euler2quat([0, 0, 0])
        # self.plane_id = p.loadURDF("plane.urdf", plane_pos, plane_ori)

        yumi_pos = self.cfgs.ARM.PYBULLET_RESET_POS
        yumi_ori = arutil.euler2quat(self.cfgs.ARM.PYBULLET_RESET_ORI)
        if self.self_collision:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       yumi_pos,
                                       yumi_ori,
                                       flags=p.URDF_USE_SELF_COLLISION)
        else:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       yumi_pos, yumi_ori)

        self._build_jnt_id()

        self.setup_single_arms(right_arm=self.right_arm,
                               left_arm=self.left_arm)

        if self.cfgs.HAS_EETOOL:
            self.eetool.activate(self.robot_id, self.jnt_to_id)
        if self.self_collision:
            # weird behavior occurs on the gripper
            # when self-collision is enforced
            if self.cfgs.HAS_EETOOL:
                self.eetool.disable_gripper_self_collision()
