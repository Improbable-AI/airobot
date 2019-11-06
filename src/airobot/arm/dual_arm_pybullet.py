    from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import pkgutil
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data
from gym.utils import seeding

import airobot.utils.common as arutil
from airobot.arm.arm import ARM
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from airobot.utils.pb_util import PB_CLIENT


class DualArmPybullet(ARM):
    """
    Class for the pybullet simulation environment
    of a dual arm robot

    """

    def __init__(self, cfgs, render=False, seed=None, self_collision=False,
                 eetool_cfg=None):
        """
        Constructor for the pybullet simulation environment
        of a dual arm robot

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            render (bool): whether to render the environment using GUI
            seed (int): random seed
            self_collision (bool): enable self_collision or
                                   not whiling loading URDF
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
        """

        self._render = render
        self.self_collision = self_collision
        self.p = p
        if eetool_cfg is None:
            eetool_cfg = {'p': self.p}
        else:
            eetool_cfg['p'] = self.p
        super(DualArmPybullet, self).__init__(cfgs=cfgs,
                                              eetool_cfg=eetool_cfg)
        self.p = PB_CLIENT

        self.np_random, _ = self._seed(seed)

        self._init_consts()
        self._init_threads()
        self._in_torque_mode = [False] * self.arm_dof

    def go_home(self, arm=None):
        """
        Move the robot to a pre-defined home pose
        """
        if arm is None:
            success = self.set_jpos(self._home_position)
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.go_home()
            elif arm == 'left':
                success = self.left_arm.go_home()
        return success

    def reset(self):
        """
        Reset the simulation environment.
        """
        raise NotImplementedError

    def step_simulation(self):
        """
        One step forward in simulation
        """
        self.p.stepSimulation()

    def realtime_simulation(self, on=True):
        """
        Turn on/off the realtime simulation mode

        Args:
            on (bool): run the simulation in realtime if True
                stop the realtime simulation if False
        """
        if on:
            self._set_step_sim(step_mode=False)
            if self._render:
                self.p.setRealTimeSimulation(1)
        else:
            self._set_step_sim(step_mode=True)
            if self._render:
                self.p.setRealTimeSimulation(0)

    def set_jpos(self, position, arm=None, joint_name=None, wait=True,
                 *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list): desired joint position(s)
            arm (str): If not provided, position should be a list and all
                actuated joints will be moved. If provided, only half the
                joints will move, corresponding to which arm was specified.
                Value should be "right" or "left".
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position. If joint_name is
                provided, then arm argument should also be provided, and
                specified joint name must correspond to joint names for the
                specified arm.
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        position = copy.deepcopy(position)
        success = False
        if arm is None:
            if len(position) != self.dual_arm_dof:
                raise ValueError('Position should contain %d'
                                 'elements if arm is not provided'
                                 % self.dual_arm_dof)
            tgt_pos = position
            self.p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        self.p.POSITION_CONTROL,
                                        targetPositions=tgt_pos,
                                        forces=self._max_torques)
            if not self._step_sim_mode and wait:
                success = wait_to_reach_jnt_goal(
                    tgt_pos,
                    get_func=self.get_jpos,
                    joint_name=joint_name,
                    get_func_derv=self.get_jvel,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_ERROR
                )
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.set_jpos(position,
                                                  joint_name=joint_name
                                                  wait=wait)
            elif arm == 'left':
                success = self.left_arm.set_jpos(position,
                                                 joint_name=joint_name
                                                 wait=wait)
        return success

    def set_jvel(self, velocity, arm=None, joint_name=None, wait=False,
                 *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies).

        Args:
            velocity (float or list): desired joint velocity(ies)
            arm (str): If not provided, velocity should be a list and all
                actuated joints will be moved. If provided, only half the
                joints will move, corresponding to which arm was specified.
                Value should be "right" or "left".
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity. If joint_name is
                provided, then arm argument should also be provided, and
                specified joint name must correspond to joint names for the
                specified arm.
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        velocity = copy.deepcopy(velocity)
        success = False
        if arm is None:
            if len(velocity) != self.dual_arm_dof:
                raise ValueError('velocity should contain %d'
                                 'elements if arm is not provided'
                                 % self.dual_arm_dof)
            tgt_vel = velocity
            self.p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        self.p.VELOCITY_CONTROL,
                                        targetVelocities=tgt_vel,
                                        forces=self._max_torques)
            if not self._step_sim_mode and wait:
                success = wait_to_reach_jnt_goal(
                    tgt_vel,
                    get_func=self.get_jvel,
                    joint_name=joint_name,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
                )
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.set_jvel(velocity,
                                                  joint_name=joint_name
                                                  wait=wait)
            elif arm == 'left':
                success = self.left_arm.set_jvel(velocity,
                                                 joint_name=joint_name
                                                 wait=wait)
        return success

    def set_jtorq(self, torque, arm=None, joint_name=None, wait=False,
                  *args, **kwargs):
        """
        Apply torque(s) to the joint(s), call enable_torque_control()
        or enable_torque_control(joint_name) before doing torque control.

        Note:
            call to this function is only effective in this simulation step.
            you need to supply torque value for each simulation step to do
            the torque control. It's easier to use torque control
            in step_simulation mode instead of realtime_simulation mode.
            If you are using realtime_simulation mode, the time interval
            between two set_jtorq() calls must be small enough (like 0.0002s)

        Args:
            torque (float or list): torque value(s) for the joint(s)
            arm (str): If not provided, torque should be a list and all
                actuated joints will be moved. If provided, only half the
                joints will move, corresponding to which arm was specified.
                Value should be "right" or "left".
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the six joints on the arm. Otherwise, only the specified joint
                will be applied with the given torque. If joint_name is
                provided, then arm argument should also be provided, and
                specified joint name must correspond to joint names for the
                specified arm.
            wait (bool): Not used in this method, just
                to keep the method signature consistent

        Returns:
            bool: Always return True as the torque will be applied as specified
            in Pybullet

        """
        success = True
        torque = copy.deepcopy(torque)
        if not self._is_in_torque_mode(joint_name):
            raise RuntimeError('Call \'enable_torque_control\' first'
                               ' before setting torque(s)')
        if arm is None:
            if len(torque) != self.dual_arm_dof:
                raise ValueError('If arm is not specified, '
                                 'Joint torques should contain'
                                 ' %d elements' % self.dual_arm_dof)
            self.p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        self.p.TORQUE_CONTROL,
                                        forces=torque)
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.set_jtorq(torque,
                                                   joint_name=joint_name
                                                   wait=wait)
            elif arm == 'left':
                success = self.left_arm.set_jtorq(torque,
                                                  joint_name=joint_name
                                                  wait=wait)
        return success

    def set_ee_pose(self, pos=None, ori=None, wait=True, arm=None,
                    *args, **kwargs):
        """
        Move the end effector to the specifed pose

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`)
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            wait (bool): Set to True if command should be blocking, otherwise
                the command can be overwritten before completion
            arm (str): Which arm to move when setting cartesian command, must
                be either "right" or "left"

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.set_ee_pose(pos=pos,
                                                     ori=ori,
                                                     wait=wait)
            elif arm == 'left':
                success = self.left_arm.set_ee_pose(pos=pos,
                                                    ori=ori,
                                                    wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, arm=None,
                    *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z
                directions (shape: :math:`[3,]`)
            eef_step (float): interpolation interval along delta_xyz.
                Interpolate a point every eef_step distance
                between the two end points
            arm (str): Which arm to move when setting cartesian command, must
                be either "right" or "left"

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        if self._step_sim_mode:
            raise AssertionError('move_ee_xyz() can '
                                 'only be called in realtime'
                                 ' simulation mode')
        if arm is None:
            raise NotImplementedError
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                success = self.right_arm.move_ee_xyz(delta_xyz=delta_xyz,
                                                     eef_step=eef_step)
            elif arm == 'left':
                success = self.left_arm.move_ee_xyz(delta_xyz=delta_xyz,
                                                    eef_step=eef_step)
        return success

    def enable_torque_control(self, joint_name=None):
        """
        Enable the torque control mode in Pybullet

        Args:
            joint_name (str): If it's none, then all the six joints
                on the UR robot are enabled in torque control mode.
                Otherwise, only the specified joint is enabled
                in torque control mode.

        """
        if joint_name is None:
            tgt_vels = [0.0] * self.arm_dof
            forces = [0.0] * self.arm_dof
            self.p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        self.p.VELOCITY_CONTROL,
                                        targetVelocities=tgt_vels,
                                        forces=forces)
            self._in_torque_mode = [True] * self.arm_dof
        else:
            jnt_id = self.jnt_to_id[joint_name]
            self.p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    self.p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0.0)
            arm_jnt_id = self.arm_jnt_names.index(joint_name)
            self._in_torque_mode[arm_jnt_id] = True

    def disable_torque_control(self, joint_name=None):
        """
        Disable the torque control mode in Pybullet

        Args:
            joint_name (str): If it's none, then all the six joints
                on the UR robot are disabled with torque control.
                Otherwise, only the specified joint is disabled with
                torque control.
                The joint(s) will enter velocity control mode.

        """
        if joint_name is None:
            self.set_jvel([0.0] * self.arm_dof)
            self._in_torque_mode = [False] * self.arm_dof
        else:
            self.set_jvel(0.0, joint_name)
            arm_jnt_id = self.arm_jnt_names.index(joint_name)
            self._in_torque_mode[arm_jnt_id] = False

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s) of the arm

        Args:
            joint_name (str, optional): If it's None,
                it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            One of the following

            - float: joint position given joint_name
            - list: joint positions if joint_name is None
              (shape: :math:`[DOF]`)
        """
        if joint_name is None:
            states = self.p.getJointStates(self.robot_id,
                                      self.arm_jnt_ids)
            pos = [state[0] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            pos = self.p.getJointState(self.robot_id, jnt_id)[0]
        return pos

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies) of the arm

        Args:
            joint_name (str, optional): If it's None, it will return
                joint velocities of all the actuated joints. Otherwise,
                it will return the joint velocity of the specified joint

        Returns:
            One of the following

            - float: joint velocity given joint_name
            - list: joint velocities if joint_name is None
              (shape: :math:`[DOF]`)
        """
        if joint_name is None:
            states = self.p.getJointStates(self.robot_id,
                                      self.arm_jnt_ids)
            vel = [state[1] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            vel = self.p.getJointState(self.robot_id, jnt_id)[1]
        return vel

    def get_jtorq(self, joint_name=None):
        """
        If the robot is operated in VELOCITY_CONTROL or POSITION_CONTROL mode,
        return the joint torque(s) applied during the last simulation step. In
        TORQUE_CONTROL, the applied joint motor torque is exactly what
        you provide, so there is no need to report it separately.
        So don't use this method to get the joint torque values when
        the robot is in TORQUE_CONTROL mode.

        Args:
            joint_name (str, optional): If it's None, it will return joint
                torques of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint

        Returns:
            One of the following

            - float: joint torque given joint_name
            - list: joint torques if joint_name is None
              (shape: :math:`[DOF]`)
        """
        if joint_name is None:
            states = self.p.getJointStates(self.robot_id, self.arm_jnt_ids)
            # state[3] is appliedJointMotorTorque
            torque = [state[3] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            torque = self.p.getJointState(self.robot_id, jnt_id)[3]
        return torque

    def get_ee_pose(self, arm=None):
        """
        Return the end effector pose

        Args:
            arm (str): Returned pose will be for specified arm, must
                be either "right" or "left"

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3,]`)
            - np.ndarray: quaternion representation of the
              EE orientation (shape: :math:`[4,]`)
            - np.ndarray: rotation matrix representation of the
              EE orientation (shape: :math:`[3, 3]`)
            - np.ndarray: euler angle representation of the
              EE orientation (roll, pitch, yaw with
              static reference frame) (shape: :math:`[3,]`)
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                return self.right_arm.get_ee_pose()
            elif arm == 'left':
                return self.left_arm.get_ee_pose()

    def get_ee_vel(self):
        """
        Return the end effector's velocity

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (shape: :math:`[3,]`)
            - np.ndarray: rotational velocity (shape: :math:`[3,]`)
        """
        info = self.p.getLinkState(self.robot_id,
                              self.ee_link_id,
                              computeLinkVelocity=1)
        trans_vel = info[6]
        rot_vel = info[7]
        return np.array(trans_vel), np.array(rot_vel)

    def compute_ik(self, pos, ori=None, arm=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`)
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[3,]`), or
                quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`).
            arm (str): Which arm EE pose corresponds to, must
                be either "right" or "left"

        Returns:
            list: solution to inverse kinematics, joint angles which achieve
            the specified EE pose (shape: :math:`[DOF]`)
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm != 'right' and arm != 'left':
                raise ValueError('Valid arm name must be specified'
                                 '("right" or "left")')
            if arm == 'right':
                return self.right_arm.compute_ik(pos=pos,
                                                 ori=ori)
            if arm == 'left':
                return self.left_arm.compute_ik(pos=pos,
                                                ori=ori)

    def _is_in_torque_mode(self, joint_name=None):
        if joint_name is None:
            return all(self._in_torque_mode)
        else:
            jnt_id = self.arm_jnt_names.index(joint_name)
            return self._in_torque_mode[jnt_id]

    def _seed(self, seed=None):
        np_random, seed = seeding.np_random(seed)
        return np_random, seed

    def _init_threads(self):
        """
        Start threads.
        One is for realtime simulation
        One is to constrain the gripper joints so
        that they move simultaneously
        """
        # realtime simulation thread
        self._set_step_sim(step_mode=False)
        if not self._render:
            self._th_sim = threading.Thread(target=self._rt_simulation)
            self._th_sim.daemon = True
            self._th_sim.start()
        else:
            self.realtime_simulation(True)

    def _init_consts(self):
        """
        Initialize constants
        """
        self._r_home_position = self.cfgs.RIGHT.ARM.HOME_POSITION
        self._l_home_position = self.cfgs.LEFT.ARM.HOME_POSITION
        self._home_position = self._r_home_position + self._l_home_position
        # joint damping for inverse kinematics
        self._ik_jd = 0.05
        self._thread_sleep = 0.001
        self.psetGravity(0, 0, -9.8)
        self.psetAdditionalSearchPath(pybullet_data.getDataPath())

        self.right_arm_jnt_names = self.cfgs.RIGHT.ARM.JOINT_NAMES
        self.left_arm_jnt_names = self.cfgs.LEFT.ARM.JOINT_NAMES
        self.arm_jnt_names = self.right_arm_jnt_names + self.left_arm_jnt_names

        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.dual_arm_dof = len(self.arm_jnt_names)
        self.single_arm_dof = self.dual_arm_dof / 2
        self.rvl_joint_names = self.arm_jnt_names + self.eetool.jnt_names
        self._ik_jds = [self._ik_jd] * len(self.rvl_joint_names)

        self.r_ee_link_jnt = self.cfgs.RIGHT.ARM.ROBOT_EE_FRAME_JOINT
        self.l_ee_link_jnt = self.cfgs.LEFT.ARM.ROBOT_EE_FRAME_JOINT

        self._r_max_torques = self.cfgs.RIGHT.ARM.MAX_TORQUES
        self._l_max_torques = self.cfgs.LEFT.ARM.MAX_TORQUES
        self._max_torques = self._r_max_torques + self._l_max_torques

    def _rt_simulation(self):
        """
        Run step simulation all the time in backend
        """
        while True:
            if not self._step_sim_mode:
                self.pstepSimulation()
            time.sleep(self._thread_sleep)

    def _build_jnt_id(self):
        """
        Build the mapping from the joint name to joint index
        """
        self.jnt_to_id = {}
        for i in range(self.pgetNumJoints(self.robot_id)):
            info = self.pgetJointInfo(self.robot_id, i)
            jnt_name = info[1].decode('UTF-8')
            self.jnt_to_id[jnt_name] = info[0]

        self.ee_link_id = self.jnt_to_id[self.ee_link_jnt]
        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]

    def _set_step_sim(self, step_mode=True):
        self._step_sim_mode = step_mode
        if self.cfgs.HAS_EETOOL:
            self.eetool._step_sim_mode = step_mode
