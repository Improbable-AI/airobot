"""
Pybullet simulation environment of a UR5e
robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import airobot.utils.common as arutil
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.utils.ikfast.franka_panda import ikfast_panda_arm
from airobot import log_warn
import numpy as np
import copy


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

        self.compute_ik_pb = self.compute_ik
        self.compute_ik = self.compute_ik_ikfast

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

    def compute_ik_ikfast(self, pos, ori=None, seed=None, *args, **kwargs):
        """Use the compiled IKFast plugin to return inverse kinematics solution
        instead of the builtin PyBullet IK functionality

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`).
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw]), (shape: :math:`[3,]`), or
                quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`).
            seed (list or np.ndarray): joint configuration to use as a
                seed for the IK solver (shape: :math:`[DOF]`).

        Returns:
            list: solution to inverse kinematics, joint angles which achieve
            the specified EE pose (shape: :math:`[DOF]`).
        """
        if seed is None:
            seed = self._home_position
            # seed = self.get_jpos()

        # convert position into the world frame
        raw_pos = copy.deepcopy(pos)
        pos = np.asarray(raw_pos) - np.asarray(self.robot_base_pos)

        if ori is not None:
            ori = arutil.to_rot_mat(ori).tolist()
        else:
            ori = arutil.to_rot_mat(self.get_ee_pose()[1]).tolist()

        if not isinstance(pos, list):
            pos = pos.tolist()

        jnt_poss_solutions = ikfast_panda_arm.get_ik(ori, pos, seed)

        if jnt_poss_solutions is None:
            log_warn('IKFast returned empty solution, defaulting to pybullet')
            jnt_poss = self.compute_ik_pb(raw_pos, ori=ori, *args, **kwargs)
        else:
            jnt_poss = self._filter_valid_joints(jnt_poss_solutions)
            if jnt_poss is None:
                log_warn('IKFast returned empty solution, defaulting to pybullet')
                jnt_poss = self.compute_ik_pb(raw_pos, ori=ori, *args, **kwargs)
        return jnt_poss

    def _filter_valid_joints(self, joint_configs):
        """Filter a list of joint configurations, returning only those
        that are valid, based on being within the joint limits of the
        arm

        Args:
            joint_configs (list): each element is a list or np.ndarray of
                joint positions, each being shape: :math:`[DOF]`

        Returns:
            list: each element is a joint configuration from the input list
            that is within joint limits
        """
        valid_joint_configs = []
        arm_jnt_ll = np.asarray(
            [self.jnt_lower_limits[i] for i in self.arm_jnt_ik_ids])
        arm_jnt_ul = np.asarray(
            [self.jnt_upper_limits[i] for i in self.arm_jnt_ik_ids])
        for config in joint_configs:
            valid = True
            valid = valid and (np.asarray(config) >= arm_jnt_ll).all()
            valid = valid and (np.asarray(config) <= arm_jnt_ul).all()
            if valid:
                valid_joint_configs.append(config)

        # get the one that is closest to the current configuration
        if len(valid_joint_configs) > 0:
            current_jpos = np.asarray(self.get_jpos())
            solutions = sorted(valid_joint_configs,
                            key=lambda q: np.linalg.norm(
                                current_jpos - np.asarray(q)))
            return solutions[0]
        else:
            return None
