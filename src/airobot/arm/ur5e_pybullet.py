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
from airobot.utils.pb_util import load_geom


class UR5ePybullet(SingleArmPybullet):
    """
    Class for the pybullet simulation environment
    of a UR5e robot with a robotiq 2f140 gripper.
    """

    def __init__(self,
                 cfgs,
                 render=False,
                 seed=None,
                 rt_simulation=True,
                 self_collision=False,
                 eetool_cfg=None):
        """
        Constructor for the pybullet simulation environment
        of a UR5e robot with a robotiq 2f140 gripper

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
        super(UR5ePybullet, self).__init__(cfgs=cfgs,
                                           render=render,
                                           seed=seed,
                                           rt_simulation=rt_simulation,
                                           self_collision=self_collision,
                                           eetool_cfg=eetool_cfg)
        self.reset()

    def reset(self):
        """
        Reset the simulation environment.
        """
        if hasattr(self, 'eetool'):
            self.eetool.deactivate()
        self.p.resetSimulation(physicsClientId=PB_CLIENT)
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_RENDERING, 0,
                                        physicsClientId=PB_CLIENT)
        self.floor_id = load_geom('box', size=[10, 10, 0.01], mass=0,
                                  base_pos=[0, 0, 0],
                                  rgba=[0.7, 0.77, 0.7, 1],
                                  specular=[1, 1, 1, 1])

        self.robot_base_pos = self.cfgs.ARM.PYBULLET_RESET_POS
        robot_base_ori = self.cfgs.ARM.PYBULLET_RESET_ORI
        self.robot_base_ori = arutil.euler2quat(robot_base_ori)
        if self.self_collision:
            colli_flag = self.p.URDF_USE_SELF_COLLISION
            self.robot_id = self.p.loadURDF(self.cfgs.PYBULLET_URDF,
                                            self.robot_base_pos,
                                            self.robot_base_ori,
                                            flags=colli_flag,
                                            physicsClientId=PB_CLIENT)
        else:
            self.robot_id = self.p.loadURDF(self.cfgs.PYBULLET_URDF,
                                            self.robot_base_pos,
                                            self.robot_base_ori,
                                            physicsClientId=PB_CLIENT)
        self._build_jnt_id()
        self.set_visual_shape()
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_RENDERING, 1,
                                        physicsClientId=PB_CLIENT)
        if hasattr(self, 'eetool'):
            self.eetool.feed_robot_info(self.robot_id, self.jnt_to_id)
            self.eetool.activate()
            if self.self_collision:
                # weird behavior occurs on the gripper
                # when self-collision is enforced
                self.eetool.disable_gripper_self_collision()

    def set_visual_shape(self):
        """
        Set the color of the UR arm
        """
        color1 = [0.25, 0.25, 0.25, 1]
        color2 = [0.95, 0.95, 0.95, 1]

        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['base-base_link_fixed_joint'],
                                 rgbaColor=color1)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['shoulder_pan_joint'],
                                 rgbaColor=color2)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['shoulder_lift_joint'],
                                 rgbaColor=color1)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['elbow_joint'],
                                 rgbaColor=color2)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['wrist_1_joint'],
                                 rgbaColor=color1)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['wrist_2_joint'],
                                 rgbaColor=color2)
        self.p.changeVisualShape(self.robot_id,
                                 self.jnt_to_id['wrist_3_joint'],
                                 rgbaColor=color1)
