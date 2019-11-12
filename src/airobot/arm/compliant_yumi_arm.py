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
import time


class CompliantYumiArm(SingleArmPybullet):
    """
    Class for the pybullet simulation environment
    of a UR5e robot with a robotiq 2f140 gripper.
    """

    def __init__(self,
                 cfgs,
                 render=False,
                 seed=None,
                 self_collision=False,
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
        super(CompliantYumiArm, self).__init__(cfgs=cfgs,
                                               render=render,
                                               seed=seed,
                                               self_collision=self_collision,
                                               eetool_cfg=eetool_cfg)
        # self.compliant_jnt_thread = threading.Thread(
        #     target=self.set_compliant_jpos)
        # self.compliant_jnt_thread.daemon = True
        # self.compliant_jnt_thread.start()

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Move the arm to the specified joint position(s). Applies regulation
        position command to the compliant joints after sending driven
        joint commands

        Args:
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        position = copy.deepcopy(position)
        success = False
        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('Position should contain %d'
                                 'elements if the joint_name'
                                 ' is not provided' % self.arm_dof)
            tgt_pos = position
            self.p.setJointMotorControlArray(self.robot_id,
                                             self.arm_jnt_ids,
                                             self.p.POSITION_CONTROL,
                                             targetPositions=tgt_pos,
                                             forces=self._max_torques,
                                             physicsClientId=PB_CLIENT)
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = position
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            self.p.setJointMotorControl2(self.robot_id,
                                         jnt_id,
                                         self.p.POSITION_CONTROL,
                                         targetPosition=tgt_pos,
                                         force=max_torque,
                                         physicsClientId=PB_CLIENT)
        self.set_compliant_jpos()
        if not self._step_sim_mode and wait:
            success = wait_to_reach_jnt_goal(
                tgt_pos,
                get_func=self.get_jpos,
                joint_name=joint_name,
                get_func_derv=self.get_jvel,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_ERROR
            )
        return success

    def set_jvel(self, velocity, joint_name=None, wait=False, *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies). Applies regulation
        position command to the compliant joints after sending driven commands

        Args:
            velocity (float or list): desired joint velocity(ies)
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits
        """
        velocity = copy.deepcopy(velocity)
        success = False
        if joint_name is None:
            velocity = copy.deepcopy(velocity)
            if len(velocity) != self.arm_dof:
                raise ValueError('Velocity should contain %d elements '
                                 'if the joint_name is not '
                                 'provided' % self.arm_dof)
            tgt_vel = velocity
            self.p.setJointMotorControlArray(self.robot_id,
                                             self.arm_jnt_ids,
                                             self.p.VELOCITY_CONTROL,
                                             targetVelocities=tgt_vel,
                                             forces=self._max_torques,
                                             physicsClientId=PB_CLIENT)
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_vel = velocity
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            self.p.setJointMotorControl2(self.robot_id,
                                         jnt_id,
                                         self.p.VELOCITY_CONTROL,
                                         targetVelocity=tgt_vel,
                                         force=max_torque,
                                         physicsClientId=PB_CLIENT)
        self.set_compliant_jpos()
        if not self._step_sim_mode and wait:
            success = wait_to_reach_jnt_goal(
                tgt_vel,
                get_func=self.get_jvel,
                joint_name=joint_name,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
            )
        return success

    def set_jtorq(self, torque, joint_name=None, wait=False, *args, **kwargs):
        """
        Apply torque(s) to the joint(s), call enable_torque_control()
        or enable_torque_control(joint_name) before doing torque control.
        Applies regulation position command to the compliant joints after
        sending driven commands

        Note:
            call to this function is only effective in this simulation step.
            you need to supply torque value for each simulation step to do
            the torque control. It's easier to use torque control
            in step_simulation mode instead of realtime_simulation mode.
            If you are using realtime_simulation mode, the time interval
            between two set_jtorq() calls must be small enough (like 0.0002s)

        Args:
            torque (float or list): torque value(s) for the joint(s)
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the six joints on the arm. Otherwise, only the specified joint
                will be applied with the given torque.
            wait (bool): Not used in this method, just
                to keep the method signature consistent

        Returns:
            bool: Always return True as the torque will be applied as specified
            in Pybullet

        """
        torque = copy.deepcopy(torque)
        if not self._is_in_torque_mode(joint_name):
            raise RuntimeError('Call \'enable_torque_control\' first'
                               ' before setting torque(s)')
        if joint_name is None:
            if len(torque) != self.arm_dof:
                raise ValueError('Joint torques should contain'
                                 ' %d elements' % self.arm_dof)
            self.p.setJointMotorControlArray(self.robot_id,
                                             self.arm_jnt_ids,
                                             self.p.TORQUE_CONTROL,
                                             forces=torque,
                                             physicsClientId=PB_CLIENT)
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise ValueError('Only torque control on'
                                 ' the arm is supported!')
            jnt_id = self.jnt_to_id[joint_name]
            self.p.setJointMotorControl2(self.robot_id,
                                         jnt_id,
                                         self.p.TORQUE_CONTROL,
                                         force=torque,
                                         physicsClientId=PB_CLIENT)
        self.set_compliant_jpos()
        return True

    def set_compliant_jpos(self):
        """
        Regulate compliant/spring like joints about nominal position
        """
        self.p.setJointMotorControlArray(
            self.robot_id,
            self.comp_jnt_ids,
            self.p.POSITION_CONTROL,
            targetPositions=[0.0]*len(self.comp_jnt_names),
            positionGains=self.comp_jnt_gains,
            physicsClientId=PB_CLIENT)

    def _init_compliant_consts(self):
        """
        Initialize additional constants relevant to compliant joints
        """
        self.comp_jnt_names = self.cfgs.ARM.COMPLIANT_JOINT_NAMES
        self.rvl_joint_names = self.arm_jnt_names + self.comp_jnt_names

        self.comp_jnt_names_set = set(self.comp_jnt_names)
        self.comp_dof = len(self.comp_jnt_names)

        self.comp_jnt_gains = self.cfgs.ARM.COMPLIANT_GAINS
        self.comp_jnt_ids = [self.jnt_to_id[jnt]
                             for jnt in self.comp_jnt_names]
