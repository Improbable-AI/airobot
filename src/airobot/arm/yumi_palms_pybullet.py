"""
Pybullet simulation environment of an ABB Yumi
robot with Gelslim palms and a compliant wrist
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy

import airobot.utils.common as arutil
from airobot.arm.dual_arm_pybullet import DualArmPybullet
from airobot.arm.single_arm_pybullet import SingleArmPybullet
from airobot.utils.arm_util import wait_to_reach_jnt_goal


class CompliantYumiArm(SingleArmPybullet):
    """
    Class for the pybullet simulation of a single
    arm of the ABB Yumi robot, with additional joints
    specified to behave like springs.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        pb_client (BulletClient): pybullet client.
        seed (int): random seed.
        self_collision (bool): enable self_collision or
            not whiling loading URDF.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.

    Attributes:
        comp_jnt_names (list): Names of the spring-like compliant joints.
        comp_dof (list): Number of spring-like compliant joints.
        comp_jnt_gains (list): Stiffness of spring-like compliant joints.
        comp_jnt_ids (list): PyBullet joint ids of compliant joints.
        max_force_comp (list): Maximum force that can be applied at
            the compliant joints.
    """

    def __init__(self,
                 cfgs,
                 pb_client,
                 seed=None,
                 self_collision=False,
                 eetool_cfg=None):
        super(CompliantYumiArm, self).__init__(cfgs=cfgs,
                                               pb_client=pb_client,
                                               seed=seed,
                                               self_collision=self_collision,
                                               eetool_cfg=eetool_cfg)

    def set_jpos(self, position, joint_name=None,
                 wait=True, ignore_physics=False, *args, **kwargs):
        """
        Move the arm to the specified joint position(s). Applies regulation
        position command to the compliant joints after sending driven
        joint commands.

        Args:
            position (float or list): desired joint position(s).
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position.
            wait (bool): whether to block the code and wait
                for the action to complete.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        position = copy.deepcopy(position)
        success = False
        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('Position should contain %d'
                                 'elements if the joint_name'
                                 ' is not provided' % self.arm_dof)
            tgt_pos = position
            if ignore_physics:
                self.set_jvel([0.] * self.arm_dof)
                for idx, jnt in enumerate(self.arm_jnt_names):
                    self.reset_joint_state(
                        jnt,
                        tgt_pos[idx]
                    )
                success = True
            else:
                self._pb.setJointMotorControlArray(self.robot_id,
                                                   self.arm_jnt_ids,
                                                   self._pb.POSITION_CONTROL,
                                                   targetPositions=tgt_pos,
                                                   forces=self._max_torques)
        else:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = position
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            if ignore_physics:
                self.set_jvel(0., joint_name)
                self.reset_joint_state(joint_name, tgt_pos)
                success = True
            else:
                self._pb.setJointMotorControl2(self.robot_id,
                                               jnt_id,
                                               self._pb.POSITION_CONTROL,
                                               targetPosition=tgt_pos,
                                               force=max_torque)
        self.set_compliant_jpos()
        if self._pb.in_realtime_mode() and wait and not ignore_physics:
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
        position command to the compliant joints after sending driven commands.

        Args:
            velocity (float or list): desired joint velocity(ies).
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity.
            wait (bool): whether to block the code and wait
                for the action to complete.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
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
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.arm_jnt_ids,
                                               self._pb.VELOCITY_CONTROL,
                                               targetVelocities=tgt_vel,
                                               forces=self._max_torques)
        else:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_vel = velocity
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            self._pb.setJointMotorControl2(self.robot_id,
                                           jnt_id,
                                           self._pb.VELOCITY_CONTROL,
                                           targetVelocity=tgt_vel,
                                           force=max_torque)
        self.set_compliant_jpos()
        if self._pb.in_realtime_mode():
            if wait:
                success = wait_to_reach_jnt_goal(
                    tgt_vel,
                    get_func=self.get_jvel,
                    joint_name=joint_name,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
                )
            else:
                success = True
        return success

    def set_jtorq(self, torque, joint_name=None, wait=False, *args, **kwargs):
        """
        Apply torque(s) to the joint(s), call enable_torque_control()
        or enable_torque_control(joint_name) before doing torque control.
        Applies regulation position command to the compliant joints after
        sending driven commands.

        Note:
            call to this function is only effective in this simulation step.
            you need to supply torque value for each simulation step to do
            the torque control. It's easier to use torque control
            in step_simulation mode instead of realtime_simulation mode.
            If you are using realtime_simulation mode, the time interval
            between two set_jtorq() calls must be small enough (like 0.0002s).

        Args:
            torque (float or list): torque value(s) for the joint(s).
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the six joints on the arm. Otherwise, only the specified joint
                will be applied with the given torque.
            wait (bool): Not used in this method, just
                to keep the method signature consistent.

        Returns:
            bool: Always return True as the torque will be applied as specified
            in Pybullet.

        """
        torque = copy.deepcopy(torque)
        if not self._is_in_torque_mode(joint_name):
            raise RuntimeError('Call \'enable_torque_control\' first'
                               ' before setting torque(s)')
        if joint_name is None:
            if len(torque) != self.arm_dof:
                raise ValueError('Joint torques should contain'
                                 ' %d elements' % self.arm_dof)
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.arm_jnt_ids,
                                               self._pb.TORQUE_CONTROL,
                                               forces=torque)
        else:
            if joint_name not in self.arm_jnt_names:
                raise ValueError('Only torque control on'
                                 ' the arm is supported!')
            jnt_id = self.jnt_to_id[joint_name]
            self._pb.setJointMotorControl2(self.robot_id,
                                           jnt_id,
                                           self._pb.TORQUE_CONTROL,
                                           force=torque)
        self.set_compliant_jpos()
        return True

    def set_compliant_jpos(self):
        """
        Regulate compliant/spring like joints about nominal position.
        """
        self._pb.setJointMotorControlArray(
            self.robot_id,
            self.comp_jnt_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=[0.0] * len(self.comp_jnt_names),
            forces=[self.max_force_comp] * len(self.comp_jnt_names),
            positionGains=self.comp_jnt_gains)

    def _init_compliant_consts(self):
        """
        Initialize additional constants relevant to compliant joints.
        """
        self.comp_jnt_names = self.cfgs.ARM.COMPLIANT_JOINT_NAMES

        self.comp_dof = len(self.comp_jnt_names)

        self.comp_jnt_gains = self.cfgs.ARM.COMPLIANT_GAINS
        self.comp_jnt_ids = [self.jnt_to_id[jnt]
                             for jnt in self.comp_jnt_names]
        self.max_force_comp = self.cfgs.ARM.COMPLIANT_MAX_FORCE


class YumiPalmsPybullet(DualArmPybullet):
    """
    Class for pybullet simulation of ABB Yumi robot with
    separate functionality for both arms, with two Gelslim
    Palms attached as end effectors instead of parallel jaw
    grippers.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        pb_client (BulletClient): pybullet client.
        seed (int): random seed.
        self_collision (bool): enable self_collision or
            not whiling loading URDF.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.

    Attributes:
        arms (dict): internal dictioanry keyed by the single arm names,
            values are interfaces to the single arm instances.
        robot_id (int): pybullet unique body id of the robot.
        left_arm (CompliantYumiArm): left arm interface.
        right_arm (CompliantYumiArm): right arm interface.
    """

    def __init__(self, cfgs, pb_client, seed=None,
                 self_collision=False,
                 eetool_cfg=None):
        super(YumiPalmsPybullet, self).__init__(cfgs=cfgs,
                                                pb_client=pb_client,
                                                seed=seed,
                                                self_collision=self_collision,
                                                eetool_cfg=eetool_cfg)
        right_cfg = cfgs.ARM.RIGHT
        left_cfg = cfgs.ARM.LEFT
        self.right_arm = CompliantYumiArm(cfgs=right_cfg,
                                          pb_client=pb_client,
                                          seed=seed,
                                          self_collision=self_collision,
                                          eetool_cfg=eetool_cfg)
        self.left_arm = CompliantYumiArm(cfgs=left_cfg,
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
        if self._first_reset or force_reset:
            self._pb.resetSimulation()

            yumi_pos = self.cfgs.ARM.PYBULLET_RESET_POS
            yumi_ori = arutil.euler2quat(self.cfgs.ARM.PYBULLET_RESET_ORI)
            if self._self_collision:
                colli_flag = {'flags': self._pb.URDF_USE_SELF_COLLISION}
                self.robot_id = self._pb.loadURDF(self.cfgs.PYBULLET_URDF,
                                                  yumi_pos,
                                                  yumi_ori,
                                                  **colli_flag)
            else:
                self.robot_id = self._pb.loadURDF(self.cfgs.PYBULLET_URDF,
                                                  yumi_pos, yumi_ori)

            self._build_jnt_id()

            self.setup_single_arms(right_arm=self.right_arm,
                                   left_arm=self.left_arm)
        else:
            self.go_home(ignore_physics=True)
        self._first_reset = False

    def setup_single_arms(self, right_arm, left_arm):
        """
        Function for setting up individual arms.

        Args:
            right_arm (CompliantYumiArm): Instance of a single
                yumi arm with compliant joints.
            left_arm (CompliantYumiArm): Instance of a single
                yumi arm with compliant joints.
        """
        self.arms[self.cfgs.ARM.RIGHT.ARM.NAME] = right_arm
        self.arms[self.cfgs.ARM.LEFT.ARM.NAME] = left_arm

        for arm in self.arms.values():
            arm.robot_id = self.robot_id
            arm._build_jnt_id()
            arm._init_compliant_consts()
