"""
Pybullet simulation environment of a UR5e
robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import airobot.utils.common as arutil
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.utils.pb_util import PB_CLIENT


class UR5ePybullet(SingleArmPybullet):
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
        super(UR5ePybullet, self).__init__(cfgs=cfgs,
                                           render=render,
                                           seed=seed,
                                           self_collision=self_collision,
                                           eetool_cfg=eetool_cfg)
        self.reset()

    def reset(self):
        """
        Reset the simulation environment.
        """
        self.eetool.deactivate()
        self.p.resetSimulation(physicsClientId=PB_CLIENT)

        plane_pos = [0, 0, 0]
        plane_ori = arutil.euler2quat([0, 0, 0])
        self.plane_id = self.p.loadURDF("plane.urdf",
                                        plane_pos,
                                        plane_ori,
                                        physicsClientId=PB_CLIENT)

        ur_pos = self.cfgs.ARM.PYBULLET_RESET_POS
        ur_ori = arutil.euler2quat(self.cfgs.ARM.PYBULLET_RESET_ORI)
        if self.self_collision:
            self.robot_id = self.p.loadURDF(self.cfgs.PYBULLET_URDF,
                                            ur_pos,
                                            ur_ori,
                                            flags=self.p.URDF_USE_SELF_COLLISION,
                                            physicsClientId=PB_CLIENT)
        else:
            self.robot_id = self.p.loadURDF(self.cfgs.PYBULLET_URDF,
                                            ur_pos, ur_ori,
                                            physicsClientId=PB_CLIENT)
        self._build_jnt_id()
        self.eetool.feed_robot_info(self.robot_id, self.jnt_to_id)
        self.eetool.activate()
        if self.self_collision:
            # weird behavior occurs on the gripper
            # when self-collision is enforced
            self.eetool.disable_gripper_self_collision()
