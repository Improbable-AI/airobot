"""
Pybullet simulation environment of a UR5e
robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import airobot.utils.common as arutil
from airobot.arm.single_arm_pybullet import SingleArmPybullet


class FrankaPybullet(SingleArmPybullet):
    """
    Class for the pybullet simulation environment
    of a Franka robot.

    Args:
        cfgs (YACS CfgNode): configurations for the arm
        pb_client (BulletClient): pybullet client
        seed (int): random seed
        self_collision (bool): enable self_collision or
                               not whiling loading URDF
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class

    Attributes:
        floor_id (int): pybullet body unique id of the floor
        robot_id (int): pybullet body unique id of the robot
        robot_base_pos (list): world frame position of the robot base link,
            shape: :math:`[3,]` ([x, y, z])
        robot_base_ori (list): world frame orientation of the robot base link
            shape: :math:`[4,]` ([x, y, z, w])
    """

    def __init__(self,
                 cfgs,
                 pb_client,
                 seed=None,
                 self_collision=False,
                 eetool_cfg=None):
        super(FrankaPybullet, self).__init__(cfgs=cfgs,
                                             pb_client=pb_client,
                                             seed=seed,
                                             self_collision=self_collision,
                                             eetool_cfg=eetool_cfg)
        self._first_reset = True
        self.reset()

    def reset(self, force_reset=False):
        """
        Reset the simulation environment.
        """
        self._pb.configureDebugVisualizer(self._pb.COV_ENABLE_RENDERING, 0)
        if self._first_reset or force_reset:
            self._pb.resetSimulation()
            self.floor_id = self._pb.load_geom('box', size=[10, 10, 0.01], mass=0,
                                               base_pos=[0, 0, 0],
                                               rgba=[0.7, 0.77, 0.7, 1],
                                               specular=[1, 1, 1, 1])

            self.robot_base_pos = self.cfgs.ARM.PYBULLET_RESET_POS
            robot_base_ori = self.cfgs.ARM.PYBULLET_RESET_ORI
            self.robot_base_ori = arutil.euler2quat(robot_base_ori).tolist()
            if self._self_collision:
                colli_flag = self._pb.URDF_USE_SELF_COLLISION
                self.robot_id = self._pb.loadURDF(self.cfgs.PYBULLET_URDF,
                                                  self.robot_base_pos,
                                                  self.robot_base_ori,
                                                  useFixedBase=True,
                                                  flags=colli_flag)
            else:
                self.robot_id = self._pb.loadURDF(self.cfgs.PYBULLET_URDF,
                                                  self.robot_base_pos,
                                                  self.robot_base_ori,
                                                  useFixedBase=True)
            self._build_jnt_id()
            # self.set_visual_shape()
            if hasattr(self, 'eetool'):
                self.eetool.feed_robot_info(self.robot_id, self.jnt_to_id)
                self.eetool.activate()
                if self._self_collision:
                    # weird behavior occurs on the gripper
                    # when self-collision is enforced
                    self.eetool.disable_gripper_self_collision()
        else:
            self.go_home(ignore_physics=True)
            if hasattr(self, 'eetool'):
                self.eetool.close(ignore_physics=True)
        self._pb.configureDebugVisualizer(self._pb.COV_ENABLE_RENDERING, 1)
        self._first_reset = False

