"""
Pybullet simulation environment of a UR5e
robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pybullet as p

import airobot.utils.common as arutil
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.arm.dual_arm_pybullet import DualArmPybullet


class YumiPybullet(DualArmPybullet):
    """
    Class for the pybullet simulation environment
    of a UR5e robot with a robotiq 2f140 gripper.
    """

    def __init__(self, cfgs, render=False, seed=None, self_collision=False,
                 eetool_cfg=None):
        """
        Constructor for the pybullet simulation environment
        of a UR5e robot with a robotiq 2f140 gripper

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
                                           eetool_cfg=eetool_cfg)
        r_cfg = cfgs.RIGHT
        l_cfg = cfgs.LEFT
        self.right_arm = SingleArmPybullet(cfgs=r_cfg,
                                           render=render,
                                           seed=seed,
                                           self_collision=self_collision,
                                           eetool_cfg=eetool_cfg)
        self.left_arm = SingleArmPybullet(cfgs=l_cfg,
                                          render=render,
                                          seed=seed,
                                          self_collision=self_collision,
                                          eetool_cfg=eetool_cfg)
        self.reset()

    def reset(self):
        """
        Reset the simulation environment.
        """
        p.resetSimulation()

        plane_pos = [0, 0, 0]
        plane_ori = arutil.euler2quat([0, 0, 0])
        self.plane_id = p.loadURDF("plane.urdf", plane_pos, plane_ori)

        ur_pos = self.cfgs.ARM.PYBULLET_RESET_POS
        ur_ori = arutil.euler2quat(self.cfgs.ARM.PYBULLET_RESET_ORI)
        if self.self_collision:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       ur_pos,
                                       ur_ori,
                                       flags=p.URDF_USE_SELF_COLLISION)
        else:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       ur_pos, ur_ori)
        self.right_arm.robot_id = self.robot_id
        self.left_arm.robot_id = self.robot_id

        self._build_jnt_id()
        self.right_arm._build_jnt_id()
        self.left_arm._build_jnt_id()

        self.eetool.activate(self.robot_id, self.jnt_to_id)
        if self.self_collision:
            # weird behavior occurs on the gripper
            # when self-collision is enforced
            self.eetool.disable_gripper_self_collision()
