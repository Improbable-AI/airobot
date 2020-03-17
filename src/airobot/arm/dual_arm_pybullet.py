from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy

from gym.utils import seeding

from airobot.arm.arm import ARM
from airobot.utils.arm_util import wait_to_reach_jnt_goal


class DualArmPybullet(ARM):
    """
    Class for the pybullet simulation environment
    of a dual arm robot.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        pb_client (BulletClient): pybullet client.
        seed (int): random seed.
        self_collision (bool): enable self_collision or
                               not whiling loading URDF.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.

    Attributes:
        cfgs (YACS CfgNode): configurations for the robot.
        robot_id (int): pybullet body unique id of the robot.
        arms (dict): internal dictionary keyed by the names of each
            single arm, with values as interfaces to the arms.
        arm_jnt_names (list): names of the arm joints.
        right_arm_jnt_names (list): names of the arm joints on the right arm.
        left_arm_jnt_names (list): names of the arm joints on the left arm.
        arm_jnt_ids (list): pybullet joint ids of the arm joints.
        r_ee_link_jnt (str): name of the end effector link on the right arm.
        l_ee_link_jnt (str): name of the end effector link on the left arm.
        dual_arm_dof (int): total number of arm joints in the robot.
        single_arm_dof (int): number of joints in a single arm of the robot.
        jnt_to_id (dict): dictionary with [joint name : pybullet joint] id
            [key : value] pairs.
        non_fixed_jnt_names (list): names of non-fixed joints in the arms,
            used for returning the correct inverse kinematics solution.

    """

    def __init__(self, cfgs, pb_client, seed=None, self_collision=False,
                 eetool_cfg=None):
        self._self_collision = self_collision
        self._pb = pb_client
        super(DualArmPybullet, self).__init__(cfgs=cfgs,
                                              eetool_cfg=eetool_cfg)

        self._np_random, _ = self._seed(seed)

        self.robot_id = None
        self.arms = {}

        self._init_consts()
        self._in_torque_mode = [False] * self.dual_arm_dof

    def setup_single_arms(self, right_arm, left_arm):
        """
        Function to setup the single arm instances, and
        maintain an internal dictionary interface to them.

        Args:
            right_arm (SingleArmPybullet): Right arm instance.
            left_arm (SingleArmPybullet): Left arm instance.
        """
        self.arms[self.cfgs.ARM.RIGHT.ARM.NAME] = right_arm
        self.arms[self.cfgs.ARM.LEFT.ARM.NAME] = left_arm

        for arm in self.arms.values():
            arm.robot_id = self.robot_id
            arm._build_jnt_id()

    def go_home(self, arm=None, ignore_physics=False):
        """
        Move the robot to a pre-defined home pose.
        """
        if arm is None:
            success = self.set_jpos(self._home_position,
                                    ignore_physics=ignore_physics)
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].go_home(ignore_physics=ignore_physics)
        return success

    def reset(self):
        """
        Reset the simulation environment.
        """
        raise NotImplementedError

    def set_jpos(self, position, arm=None, joint_name=None,
                 wait=True, ignore_physics=False, *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list): desired joint position(s).
            arm (str): If not provided, position should be a list and all
                actuated joints will be moved. If provided, only half the
                joints will move, corresponding to which arm was specified.
                Value should match arm names in cfg file.
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position. If joint_name is
                provided, then arm argument should also be provided, and
                specified joint name must correspond to joint names for the
                specified arm.
            wait (bool): whether to block the code and wait
                for the action to complete.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        position = copy.deepcopy(position)
        success = False
        if arm is None:
            if len(position) != self.dual_arm_dof:
                raise ValueError('Position should contain %d '
                                 'elements if arm is not provided'
                                 % self.dual_arm_dof)
            tgt_pos = position
            if ignore_physics:
                self.set_jvel([0.] * self.dual_arm_dof)
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
            if self._pb.in_realtime_mode() and wait and not ignore_physics:
                success = wait_to_reach_jnt_goal(
                    tgt_pos,
                    get_func=self.get_jpos,
                    joint_name=joint_name,
                    get_func_derv=self.get_jvel,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_ERROR
                )
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].set_jpos(position,
                                              joint_name=joint_name,
                                              ignore_physics=ignore_physics,
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
                Value should match arm names in cfg file.
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
                raise ValueError('velocity should contain %d '
                                 'elements if arm is not provided'
                                 % self.dual_arm_dof)
            tgt_vel = velocity
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.arm_jnt_ids,
                                               self._pb.VELOCITY_CONTROL,
                                               targetVelocities=tgt_vel,
                                               forces=self._max_torques)
            if self._pb.in_realtime_mode() and wait:
                success = wait_to_reach_jnt_goal(
                    tgt_vel,
                    get_func=self.get_jvel,
                    joint_name=joint_name,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
                )
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].set_jvel(velocity,
                                              joint_name=joint_name,
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
            torque (float or list): torque value(s) for the joint(s).
            arm (str): If not provided, torque should be a list and all
                actuated joints will be moved. If provided, only half the
                joints will move, corresponding to which arm was specified.
                Value should match arm names in cfg file.
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the six joints on the arm. Otherwise, only the specified joint
                will be applied with the given torque. If joint_name is
                provided, then arm argument should also be provided, and
                specified joint name must correspond to joint names for the
                specified arm.
            wait (bool): Not used in this method, just
                to keep the method signature consistent.

        Returns:
            bool: Always return True as the torque will be applied as specified
            in Pybullet.

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
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.arm_jnt_ids,
                                               self._pb.TORQUE_CONTROL,
                                               forces=torque)
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].set_jtorq(torque,
                                               joint_name=joint_name,
                                               wait=wait)
        return success

    def set_ee_pose(self, pos=None, ori=None, wait=True, arm=None,
                    *args, **kwargs):
        """
        Move the end effector to the specifed pose.

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`).
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation.
            wait (bool): Set to True if command should be blocking, otherwise
                the command can be overwritten before completion.
            arm (str): Which arm to move when setting cartesian command, must
                match arm names in cfg file.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].set_ee_pose(pos=pos,
                                                 ori=ori,
                                                 wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, arm=None,
                    *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation.

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z
                directions (shape: :math:`[3,]`).
            eef_step (float): interpolation interval along delta_xyz.
                Interpolate a point every eef_step distance
                between the two end points.
            arm (str): Which arm to move when setting cartesian command, must
                match arm names in cfg file.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        if not self._pb.in_realtime_mode():
            raise AssertionError('move_ee_xyz() can '
                                 'only be called in realtime'
                                 ' simulation mode')
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            success = self.arms[arm].move_ee_xyz(delta_xyz=delta_xyz,
                                                 eef_step=eef_step)
        return success

    def enable_torque_control(self, joint_name=None):
        """
        Enable the torque control mode in Pybullet.

        Args:
            joint_name (str): If it's none, then all the six joints
                on the UR robot are enabled in torque control mode.
                Otherwise, only the specified joint is enabled
                in torque control mode.

        """
        if joint_name is None:
            for arm in self.arms.values():
                arm.enable_torque_control()
                self._in_torque_mode = [True] * self.dual_arm_dof
        else:
            arm = self._check_arm(joint_name)
            self.arms[arm].enable_torque_control(joint_name)
            arm_jnt_id = self.arm_jnt_names.index(joint_name)
            self._in_torque_mode[arm_jnt_id] = True

    def disable_torque_control(self, joint_name=None):
        """
        Disable the torque control mode in Pybullet.

        Args:
            joint_name (str): If it's none, then all the six joints
                on the UR robot are disabled with torque control.
                Otherwise, only the specified joint is disabled with
                torque control.
                The joint(s) will enter velocity control mode.

        """
        if joint_name is None:
            for arm in self.arms.values():
                arm.disable_torque_control()
                self._in_torque_mode = [False] * self.dual_arm_dof
        else:
            arm = self._check_arm(joint_name)
            self.arms[arm].disable_torque_control(joint_name)
            arm_jnt_id = self.arm_jnt_names.index(joint_name)
            self._in_torque_mode[arm_jnt_id] = False

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s) of the arm.

        Args:
            joint_name (str, optional): If it's None,
                it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint.

        Returns:
            One of the following

            - float: joint position given joint_name.
            - list: joint positions if joint_name is None
              (shape: :math:`[DOF]`).
        """
        if joint_name is None:
            states = self._pb.getJointStates(self.robot_id,
                                             self.arm_jnt_ids)
            pos = [state[0] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            pos = self._pb.getJointState(self.robot_id,
                                         jnt_id)[0]
        return pos

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies) of the arm.

        Args:
            joint_name (str, optional): If it's None, it will return
                joint velocities of all the actuated joints. Otherwise,
                it will return the joint velocity of the specified joint.

        Returns:
            One of the following

            - float: joint velocity given joint_name.
            - list: joint velocities if joint_name is None
              (shape: :math:`[DOF]`).
        """
        if joint_name is None:
            states = self._pb.getJointStates(self.robot_id,
                                             self.arm_jnt_ids)
            vel = [state[1] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            vel = self._pb.getJointState(self.robot_id,
                                         jnt_id)[1]
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
                return the joint torque of the specified joint.

        Returns:
            One of the following

            - float: joint torque given joint_name.
            - list: joint torques if joint_name is None
              (shape: :math:`[DOF]`).
        """
        if joint_name is None:
            states = self._pb.getJointStates(self.robot_id,
                                             self.arm_jnt_ids)
            # state[3] is appliedJointMotorTorque
            torque = [state[3] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            torque = self._pb.getJointState(self.robot_id,
                                            jnt_id)[3]
        return torque

    def get_ee_pose(self, arm=None):
        """
        Return the end effector pose.

        Args:
            arm (str): Returned pose will be for specified arm, must
                match arm names in cfg file.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3,]`).
            - np.ndarray: quaternion representation of the
              EE orientation (shape: :math:`[4,]`).
            - np.ndarray: rotation matrix representation of the
              EE orientation (shape: :math:`[3, 3]`).
            - np.ndarray: euler angle representation of the
              EE orientation (roll, pitch, yaw with
              static reference frame) (shape: :math:`[3,]`).
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            return self.arms[arm].get_ee_pose()

    def get_ee_vel(self, arm=None):
        """
        Return the end effector's velocity.

        Args:
            arm (str): Which arm to get velocity for,
                must match arm names in cfg file.

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (shape: :math:`[3,]`).
            - np.ndarray: rotational velocity (shape: :math:`[3,]`).
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            return self.arms[arm].get_ee_vel()

    def compute_ik(self, pos, ori=None, arm=None, ns=False, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector.

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`)
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[3,]`), or
                quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`).
            arm (str): Which arm EE pose corresponds to, must
                match arm names in cfg file
            ns (bool): whether to use the nullspace options in pybullet,
                True if nullspace should be used. Defaults to False.

        Returns:
            list: solution to inverse kinematics, joint angles which achieve
            the specified EE pose (shape: :math:`[DOF]`).
        """
        if arm is None:
            raise NotImplementedError
        else:
            if arm not in self.arms:
                raise ValueError('Valid arm name must be specified '
                                 '("%s" or "%s")'
                                 % (self._arm_names[0], self._arm_names[1]))
            return self.arms[arm].compute_ik(pos=pos, ori=ori, ns=ns)

    def _check_arm(self, joint_name):
        """
        Checks which arm a joint is part of

        Args:
            joint_name (str): Name of the joint to check

        Returns:
            str: Name of the arm that has the specified joint
        """
        if joint_name in self.right_arm_jnt_names:
            arm_name = self._r_arm_name
        elif joint_name in self.left_arm_jnt_names:
            arm_name = self._l_arm_name
        else:
            raise ValueError('Joint name not recognized')
        return arm_name

    def reset_joint_state(self, jnt_name, jpos, jvel=0):
        """
        Reset the state of the joint. It's best only to do
        this at the start, while not running the simulation.
        It will overrides all physics simulation.

        Args:
            jnt_name (str): joint name.
            jpos (float): target joint position.
            jvel (float): optional, target joint velocity.

        """
        self._pb.resetJointState(self.robot_id,
                                 self.jnt_to_id[jnt_name],
                                 targetValue=jpos,
                                 targetVelocity=jvel)

    def _is_in_torque_mode(self, joint_name=None):
        if joint_name is None:
            return all(self._in_torque_mode)
        else:
            jnt_id = self.arm_jnt_names.index(joint_name)
            return self._in_torque_mode[jnt_id]

    def _seed(self, seed=None):
        np_random, seed = seeding.np_random(seed)
        return np_random, seed

    def _init_consts(self):
        """
        Initialize constants
        """
        self._r_arm_name = self.cfgs.ARM.RIGHT.ARM.NAME
        self._l_arm_name = self.cfgs.ARM.LEFT.ARM.NAME
        self._arm_names = [self._r_arm_name, self._l_arm_name]

        self._r_home_position = self.cfgs.ARM.RIGHT.ARM.HOME_POSITION
        self._l_home_position = self.cfgs.ARM.LEFT.ARM.HOME_POSITION
        self._home_position = self._r_home_position + self._l_home_position

        self.right_arm_jnt_names = self.cfgs.ARM.RIGHT.ARM.JOINT_NAMES
        self.left_arm_jnt_names = self.cfgs.ARM.LEFT.ARM.JOINT_NAMES
        self.arm_jnt_names = self.right_arm_jnt_names + self.left_arm_jnt_names

        self.dual_arm_dof = len(self.arm_jnt_names)
        self.single_arm_dof = int(self.dual_arm_dof / 2)

        self.r_ee_link_jnt = self.cfgs.ARM.RIGHT.ARM.ROBOT_EE_FRAME_JOINT
        self.l_ee_link_jnt = self.cfgs.ARM.LEFT.ARM.ROBOT_EE_FRAME_JOINT

        self._r_max_torques = self.cfgs.ARM.RIGHT.ARM.MAX_TORQUES
        self._l_max_torques = self.cfgs.ARM.LEFT.ARM.MAX_TORQUES
        self._max_torques = self._r_max_torques + self._l_max_torques

    def _build_jnt_id(self):
        """
        Build the mapping from the joint name to joint index.
        """
        self.jnt_to_id = {}
        self.non_fixed_jnt_names = []
        for i in range(self._pb.getNumJoints(self.robot_id)):
            info = self._pb.getJointInfo(self.robot_id, i)
            jnt_name = info[1].decode('UTF-8')
            self.jnt_to_id[jnt_name] = info[0]
            if info[2] != self._pb.JOINT_FIXED:
                self.non_fixed_jnt_names.append(jnt_name)
        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]
