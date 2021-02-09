from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy

import numpy as np
from gym.utils import seeding

import airobot.utils.common as arutil
import airobot.utils.transform_util as pose_util
from airobot.arm.arm import ARM
from airobot.utils.arm_util import wait_to_reach_jnt_goal


class SingleArmPybullet(ARM):
    """
    Base class for a single arm simulated in pybullet.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        pb_client (BulletClient): pybullet client.
        seed (int): random seed.
        self_collision (bool): enable self_collision or
            not whiling loading URDF.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.

    Attributes:
        cfgs (YACS CfgNode): configurations for the arm.
        robot_id (int): pybullet body unique id of the robot.
        arm_jnt_names (list): names of the arm joints.
        arm_dof (int): degrees of freedom of the arm.
        arm_jnt_ids (list): pybullet joint ids of the arm joints.
        arm_jnt_ik_ids (list): joint indices of the non-fixed arm joints.
        ee_link_jnt (str): joint name of the end-effector link.
        ee_link_id (int): joint id of the end-effector link.
        jnt_to_id (dict): dictionary with [joint name : pybullet joint] id
            [key : value] pairs.
        non_fixed_jnt_names (list): names of non-fixed joints in the arms,
            used for returning the correct inverse kinematics solution.

    """

    def __init__(self,
                 cfgs,
                 pb_client,
                 seed=None,
                 self_collision=False,
                 eetool_cfg=None):
        self._self_collision = self_collision
        self._pb = pb_client
        super(SingleArmPybullet, self).__init__(cfgs=cfgs,
                                                eetool_cfg=eetool_cfg)
        self.robot_id = None
        self._np_random, _ = self._seed(seed)

        self._init_consts()
        self._in_torque_mode = [False] * self.arm_dof

    def go_home(self, ignore_physics=False):
        """
        Move the robot to a pre-defined home pose
        """
        success = self.set_jpos(self._home_position,
                                ignore_physics=ignore_physics)
        return success

    def reset(self):
        """
        Reset the simulation environment.
        """
        if self.robot_id is None:
            self.robot_id = self._pb.loadURDF(self.cfgs.PYBULLET_URDF,
                                              [0, 0, 0], [0, 0, 0, 1])

    def set_jpos(self, position, joint_name=None,
                 wait=True, ignore_physics=False, *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list): desired joint position(s).
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position.
            wait (bool): whether to block the code and wait
                for the action to complete.
            ignore_physics (bool): hard reset the joints to the target joint
                positions. It's best only to do this at the start,
                while not running the simulation. It will overrides
                all physics simulation.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        position = copy.deepcopy(position)
        success = False
        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('Position should contain %d '
                                 'elements if the joint_name'
                                 ' is not provided' % self.arm_dof)
            tgt_pos = position
            if ignore_physics:
                # we need to set the joints to velocity control mode
                # so that the reset takes effect. Otherwise, the joints
                # will just go back to the original positions
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
        Move the arm with the specified joint velocity(ies).

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

        Note:
            call to this function is only effective in this simulation step.
            you need to supply torque value for each simulation step to do
            the torque control. It's easier to use torque control
            in step_simulation mode instead of realtime_simulation mode.
            If you are using realtime_simulation mode, the time interval
            between two set_jtorq() calls must be small enough (like 0.0002s)

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
        return True

    def set_ee_pose(self, pos=None, ori=None, wait=True, *args, **kwargs):
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

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        if pos is None:
            pose = self.get_ee_pose()
            pos = pose[0]
        jnt_pos = self.compute_ik(pos, ori)
        success = self.set_jpos(jnt_pos, wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation.

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z
                directions (shape: :math:`[3,]`).
            eef_step (float): interpolation interval along delta_xyz.
                Interpolate a point every eef_step distance
                between the two end points.

        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.
        """
        if not self._pb.in_realtime_mode():
            raise AssertionError('move_ee_xyz() can '
                                 'only be called in realtime'
                                 ' simulation mode')
        pos, quat, rot_mat, euler = self.get_ee_pose()
        cur_pos = np.array(pos)
        delta_xyz = np.array(delta_xyz)

        waypoints = arutil.linear_interpolate_path(cur_pos,
                                                   delta_xyz,
                                                   eef_step)
        way_jnt_positions = []
        for i in range(waypoints.shape[0]):
            tgt_jnt_poss = self.compute_ik(waypoints[i, :].flatten().tolist(),
                                           quat)
            way_jnt_positions.append(copy.deepcopy(tgt_jnt_poss))
        success = False
        for jnt_poss in way_jnt_positions:
            success = self.set_jpos(jnt_poss, **kwargs)
        return success

    def rot_ee_xyz(self, angle, axis='x', N=50, *args, **kwargs):
        """
        Rotate the end-effector about one of the end-effector axes,
        without changing the position

        Args:
            angle (float): Angle by which to rotate in radians.
            axis (str): Which end-effector frame axis to rotate about.
                Must be in ['x', 'y', 'z'].
            N (int): Number of waypoints along the rotation trajectory
                (larger N means motion will be more smooth but potentially
                slower).
        Returns:
            bool: A boolean variable representing if the action is successful
            at the moment when the function exits.            
        """
        if not self._pb.in_realtime_mode():
            raise AssertionError('rot_ee_xyz() can  '
                                 'only be called in realtime'
                                 ' simulation mode')

        axis_dict = {'x': 0, 'y': 1, 'z': 2}
        if axis not in axis_dict.keys(): 
            raise ValueError('axis must be in [x, y, z]')

        pos, quat = self.get_ee_pose()[:2]
        euler_rot = [0.0]*3
        euler_rot[axis_dict[axis]] = angle

        transformation = np.eye(4)
        transformation[:-1, :-1] = arutil.euler2rot(euler_rot)

        current_pose = pose_util.list2pose_stamped(pos.tolist() + quat.tolist())

        new_pose = pose_util.transform_body(
            current_pose,
            pose_util.pose_from_matrix(transformation)
        )

        waypoint_poses = pose_util.interpolate_pose(current_pose, new_pose, N=N)

        for waypoint in waypoint_poses:
            pose = pose_util.pose_stamped2list(waypoint)
            success = self.set_ee_pose(pose[:3], pose[3:])
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
            tgt_vels = [0.0] * self.arm_dof
            forces = [0.0] * self.arm_dof
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.arm_jnt_ids,
                                               self._pb.VELOCITY_CONTROL,
                                               targetVelocities=tgt_vels,
                                               forces=forces)
            self._in_torque_mode = [True] * self.arm_dof
        else:
            jnt_id = self.jnt_to_id[joint_name]
            self._pb.setJointMotorControl2(self.robot_id,
                                           jnt_id,
                                           self._pb.VELOCITY_CONTROL,
                                           targetVelocity=0,
                                           force=0.0)
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
            self.set_jvel([0.0] * self.arm_dof)
            self._in_torque_mode = [False] * self.arm_dof
        else:
            self.set_jvel(0.0, joint_name)
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
            joint_name (str, optional): If it's None,
                it will return joint torques
                of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint.

        Returns:
            One of the following

            - float: joint torque given joint_name
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

    def get_ee_pose(self):
        """
        Return the end effector pose.

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
        info = self._pb.getLinkState(self.robot_id, self.ee_link_id)
        pos = info[4]
        quat = info[5]

        rot_mat = arutil.quat2rot(quat)
        euler = arutil.quat2euler(quat, axes='xyz')  # [roll, pitch, yaw]
        return np.array(pos), np.array(quat), rot_mat, euler

    def get_ee_vel(self):
        """
        Return the end effector's velocity.

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (shape: :math:`[3,]`).
            - np.ndarray: rotational velocity (shape: :math:`[3,]`).
        """
        info = self._pb.getLinkState(self.robot_id,
                                     self.ee_link_id,
                                     computeLinkVelocity=1)
        trans_vel = info[6]
        rot_vel = info[7]
        return np.array(trans_vel), np.array(rot_vel)

    def compute_ik(self, pos, ori=None, ns=False, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector.

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`).
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[3,]`), or
                quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`).
            ns (bool): whether to use the nullspace options in pybullet,
                True if nullspace should be used. Defaults to False.

        Returns:
            list: solution to inverse kinematics, joint angles which achieve
            the specified EE pose (shape: :math:`[DOF]`).
        """
        kwargs.setdefault('jointDamping', self._ik_jds)
        if ns:
            kwargs.setdefault('lowerLimits', self.jnt_lower_limits)
            kwargs.setdefault('upperLimits', self.jnt_upper_limits)
            kwargs.setdefault('jointRanges', self.jnt_ranges)
            kwargs.setdefault('restPoses', self.jnt_rest_poses)

        if ori is not None:
            ori = arutil.to_quat(ori)
            jnt_poss = self._pb.calculateInverseKinematics(self.robot_id,
                                                           self.ee_link_id,
                                                           pos,
                                                           ori,
                                                           **kwargs)
        else:
            jnt_poss = self._pb.calculateInverseKinematics(self.robot_id,
                                                           self.ee_link_id,
                                                           pos,
                                                           **kwargs)
        jnt_poss = list(map(arutil.ang_in_mpi_ppi, jnt_poss))
        arm_jnt_poss = [jnt_poss[i] for i in self.arm_jnt_ik_ids]
        return arm_jnt_poss

    def _get_joint_ranges(self):
        """
        Return a default set of values for the arguments to IK
        with nullspace turned on. Returns joint ranges from the
        URDF and the current value of each joint angle for the
        rest poses.

        Returns:
            4-element tuple containing:

            - list: list of lower limits for each joint (shape: :math:`[DOF]`).
            - list: list of upper limits for each joint (shape: :math:`[DOF]`).
            - list: list of joint ranges for each joint (shape: :math:`[DOF]`).
            - list: list of rest poses (shape: :math:`[DOF]`).
        """
        ll, ul, jr, rp = [], [], [], []

        for i in range(self._pb.getNumJoints(self.robot_id)):
            info = self._pb.getJointInfo(self.robot_id, i)
            if info[3] > -1:
                lower, upper = info[8:10]
                j_range = upper - lower

                rest_pose = self._pb.getJointState(
                    self.robot_id,
                    i)[0]

                ll.append(lower)
                ul.append(upper)
                jr.append(j_range)
                rp.append(rest_pose)

        return ll, ul, jr, rp

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
        self._home_position = self.cfgs.ARM.HOME_POSITION
        # joint damping for inverse kinematics
        self._ik_jd = self.cfgs.ARM.PYBULLET_IK_DAMPING
        self._max_torques = self.cfgs.ARM.MAX_TORQUES
        self.arm_jnt_names = self.cfgs.ARM.JOINT_NAMES

        self.arm_dof = len(self.arm_jnt_names)
        self.ee_link_jnt = self.cfgs.ARM.ROBOT_EE_FRAME_JOINT

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

        self._ik_jds = [self._ik_jd] * len(self.non_fixed_jnt_names)
        self.ee_link_id = self.jnt_to_id[self.ee_link_jnt]
        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]
        self.arm_jnt_ik_ids = [self.non_fixed_jnt_names.index(jnt)
                               for jnt in self.arm_jnt_names]
        ll, ul, jr, rp = self._get_joint_ranges()
        self.jnt_lower_limits = ll
        self.jnt_upper_limits = ul
        self.jnt_ranges = jr
        self.jnt_rest_poses = rp
