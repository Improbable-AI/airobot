"""
Pybullet simulation environment of a UR5e
 robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data
from gym.utils import seeding

import airobot.utils.common as arutil
from airobot.robot.robot import Robot
from airobot.sensor.camera.pybullet_cam import PyBulletCamera


# import pkgutil


class ABBYumiPyBullet(Robot):
    """
    Class for the pybullet simulation environment
        of a UR5e robot with a robotiq 2f140 gripper.
    """

    def __init__(self, cfgs, render=False, seed=None, self_collision=False):
        """
        Constructor for the pybullet simulation environment
        of a UR5e robot with a robotiq 2f140 gripper

        Args:
            cfgs (YACS CfgNode): configurations for the robot
            render (bool): whether to render the environment using GUI
            seed (int): random seed
            self_collision (bool): enable self_collision or
                                   not whiling loading URDF
        """
        super(ABBYumiPyBullet, self).__init__(cfgs=cfgs)
        self._render = render
        self.self_collision = self_collision
        self.p = p
        if self._render:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
            # TODO Check if EGL is working properly
            #  EGL rendering seems to give different images
            # # using the eglRendererPlugin (hardware OpenGL acceleration)
            # egl = pkgutil.get_loader('eglRenderer')
            # if egl:
            #     p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            # else:
            #     p.loadPlugin("eglRendererPlugin")
        self.np_random, _ = self._seed(seed)
        self._init_consts()
        self.reset()
        self._init_threads()

    def __del__(self):
        p.disconnect()

    def go_home(self):
        """
        Move the robot to a pre-defined home pose
        """
        success = self.set_jpos(self._home_position)
        return success

    def reset(self):
        """
        Reset the simulation environment.
        """
        p.resetSimulation()

        plane_pos = [0, 0, 0]
        plane_ori = arutil.euler2quat([0, 0, 0])
        self.plane_id = p.loadURDF("plane.urdf", plane_pos, plane_ori)

        # align world frame with yumi body
        yumi_pos = self.cfgs.BASE_FRAME_OFFSET
        yumi_ori = arutil.euler2quat([0, 0, 0])
        if self.self_collision:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       yumi_pos,
                                       yumi_ori,
                                       flags=p.URDF_USE_SELF_COLLISION)
        else:
            self.robot_id = p.loadURDF(
                self.cfgs.PYBULLET_URDF, yumi_pos, yumi_ori)
        self._build_jnt_id()
        if self.self_collision:
            pass

    def step_simulation(self):
        """
        One step forward in simulation
        """
        p.stepSimulation()

    def realtime_simulation(self, on=True):
        """
        Turn on/off the realtime simulation mode

        Args:
            on (bool): run the simulation in realtime if True
                stop the realtime simulation if False
        """
        if on:
            self._step_sim_mode = False
            if self._render:
                p.setRealTimeSimulation(1)
        else:
            self._step_sim_mode = True
            if self._render:
                p.setRealTimeSimulation(0)

    def load_object(self, urdf, pos, ori):
        p.loadURDF(
            urdf,
            pos,
            ori,
            flags=p.URDF_USE_SELF_COLLISION
        )

    def set_jpos(self, position, joint_name=None, wait=True, arm=None,
                 *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        position = copy.deepcopy(position)
        success = False
        if joint_name is None:
            if len(position) == 7:
                if arm is None or (arm != 'right' and arm != 'left'):
                    raise ValueError('If specifying only 7 joint angles, arm'
                                     'must be specified')
                if arm == 'right':
                    left_j_pos = self.get_jpos()[7:]
                    position = position + left_j_pos
                elif arm == 'left':
                    right_j_pos = self.get_jpos()[:7]
                    position = right_j_pos + position
            if len(position) != 14:
                raise ValueError('Position should contain 14'
                                 'elements if the joint_name is not provided')
            tgt_pos = position
            p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        p.POSITION_CONTROL,
                                        targetPositions=tgt_pos,
                                        forces=self._max_torques[:len(self.arm_jnt_names)])
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = position
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.POSITION_CONTROL,
                                    targetPosition=tgt_pos,
                                    force=max_torque)
        if not self._step_sim_mode and wait:
            success = self._wait_to_reach_jnt_goal(tgt_pos,
                                                   joint_name=joint_name,
                                                   mode='pos')
        return success

    def set_jvel(self, velocity, joint_name=None, wait=False, arm=None,
                 *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies).

        Args:
            velocity (float or list): desired joint velocity(ies)
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        velocity = copy.deepcopy(velocity)
        success = False
        if joint_name is None:
            velocity = copy.deepcopy(velocity)
            if len(velocity) == 7:
                if arm is None or (arm != 'right' and arm != 'left'):
                    raise ValueError('If specifying only 7 joint angles, arm'
                                     'must be specified')
                if arm == 'right':
                    left_j_vel = [0.0] * 7
                    velocity = velocity + left_j_vel
                elif arm == 'left':
                    right_j_vel = [0.0] * 7
                    velocity = right_j_vel + velocity
            if len(velocity) != 14:
                raise ValueError('Velocity should contain 14'
                                 'elements if the joint_name is not provided')
            tgt_vel = velocity
            p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        p.VELOCITY_CONTROL,
                                        targetVelocities=tgt_vel,
                                        forces=self._max_torques[:len(self.arm_jnt_names)])
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_vel = velocity
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                max_torque = self._max_torques[arm_jnt_idx]
                jnt_id = self.jnt_to_id[joint_name]
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=tgt_vel,
                                    force=max_torque)
        if not self._step_sim_mode and wait:
            success = self._wait_to_reach_jnt_goal(tgt_vel,
                                                   joint_name=joint_name,
                                                   mode='vel')
        return success

    def set_jtorq(self, torque, joint_name=None, wait=False, arm=None,
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
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the six joints on the arm. Otherwise, only the specified joint
                will be applied with the given torque.
            wait (bool): Not used in this method, just
                to keep the method signature consistent

        Returns:
            Always return True as the torque will be applied as specified
            in Pybullet

        """
        torque = copy.deepcopy(torque)
        if joint_name is None:
            if len(torque) == 7:
                if arm is None or (arm != 'right' and arm != 'left'):
                    raise ValueError('If specifying only 7 joint angles, arm'
                                     'must be specified')
                if arm == 'right':
                    left_j_torque = self.get_jtorq()[7:]
                    torque = torque + left_j_torque
                elif arm == 'left':
                    right_j_torque = self.get_jtorq()[:7]
                    torque = right_j_torque + torque
            if len(torque) != 14:
                raise ValueError('Joint torques should contain 6 elements')
            p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        p.TORQUE_CONTROL,
                                        forces=torque)
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise ValueError('Only torque control on'
                                 ' the arm is supported!')
            jnt_id = self.jnt_to_id[joint_name]
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.TORQUE_CONTROL,
                                    force=torque)
        return True

    def set_ee_pose(self, pos, ori=None, wait=True, arm='right',
                    *args, **kwargs):
        """
        Move the end effector to the specifed pose
        Args:
            pos (list or np.ndarray): position (shape: [3,])
            ori (list or np.ndarray): orientation. It can be rotation matrix (shape: [3,3]),
                quaternion ([qx, qy, qz, qw], shape: [4,]),
                or euler angles ([roll, pitch, yaw], shape: [3,])

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        if arm != 'right' and arm != 'left':
            arm = 'right'
        jnt_pos = self.compute_ik(pos, ori, arm=arm)
        success = self.set_jpos(jnt_pos, arm=arm, wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, arm='right', *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z directions (shape: [3,])
            eef_step (float): interpolation interval along delta_xyz. Interpolate
                a point every eef_step distance between the two end points

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        if arm != 'right' and arm != 'left':
            arm = 'right'
        if self._step_sim_mode:
            raise AssertionError('move_ee_xyz() can '
                                 'only be called in realtime'
                                 ' simulation mode')
        pos, quat, rot_mat, euler = self.get_ee_pose(arm=arm)
        cur_pos = np.array(pos)
        delta_xyz = np.array(delta_xyz)
        path_len = np.linalg.norm(delta_xyz)
        num_pts = int(np.ceil(path_len / float(eef_step)))
        if num_pts <= 1:
            num_pts = 2
        waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
        waypoints = cur_pos + waypoints_sp / float(path_len) * delta_xyz
        way_jnt_positions = []
        for i in range(waypoints.shape[0]):
            tgt_jnt_poss = self.compute_ik(waypoints[i, :].flatten().tolist(),
                                           quat, arm=arm)
            way_jnt_positions.append(copy.deepcopy(tgt_jnt_poss))
        success = False
        for jnt_poss in way_jnt_positions:
            success = self.set_jpos(jnt_poss, arm=arm)
        return success

    def enable_torque_control(self, joint_name=None):
        """
        Enable the torque control mode in Pybullet

        Args:
            joint_name: If it's none, then all the six joints
                on the UR robot are enabled in torque control mode.
                Otherwise, only the specified joint is enabled
                in torque control mode.

        """
        if joint_name is None:
            tgt_vels = [0.0] * len(self.arm_jnt_ids)
            forces = [0.0] * len(self.arm_jnt_ids)
            p.setJointMotorControlArray(self.robot_id,
                                        self.arm_jnt_ids,
                                        p.VELOCITY_CONTROL,
                                        targetVelocities=tgt_vels,
                                        forces=forces)
        else:
            jnt_id = self.jnt_to_id[joint_name]
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0.0)

    def disable_torque_control(self, joint_name=None):
        """
        Disable the torque control mode in Pybullet

        Args:
            joint_name: If it's none, then all the six joints
                on the UR robot are disabled with torque control.
                Otherwise, only the specified joint is disabled with
                torque control.
                The joint(s) will enter velocity control mode.

        """
        if joint_name is None:
            self.set_jvel([0.0] * 6)
        else:
            self.set_jvel(0.0, joint_name)

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s) of the arm

        Args:
            joint_name: If it's None, it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            joint position (float) or joint positions (list) depends on
            the joint_name
        """
        if joint_name is None:
            states = p.getJointStates(self.robot_id, self.arm_jnt_ids)
            pos = [state[0] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            pos = p.getJointState(self.robot_id, jnt_id)[0]
        return pos

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies) of the arm

        Args:
            joint_name: If it's None, it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint velocity of the specified joint

        Returns:
            joint velocity (float) or joint velocities (list) depends on
            the joint_name
        """
        if joint_name is None:
            states = p.getJointStates(self.robot_id, self.arm_jnt_ids)
            vel = [state[1] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            vel = p.getJointState(self.robot_id, jnt_id)[1]
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
            joint_name: If it's None, it will return joint torques
                of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint

        Returns:
            joint torque (float) or joint torques (list) depends on
            the joint_name
        """
        if joint_name is None:
            states = p.getJointStates(self.robot_id, self.arm_jnt_ids)
            # state[3] is appliedJointMotorTorque
            torque = [state[3] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            torque = p.getJointState(self.robot_id, jnt_id)[3]
        return torque

    def get_ee_pose(self, arm='right'):
        """
        Return the end effector pose

        Returns:
            np.ndarray: x, y, z position of the EE (shape: [3,])
            np.ndarray: quaternion representation of the EE orientation (shape: [4,])
            np.ndarray: rotation matrix representation of the EE orientation (shape: [3, 3])
            np.ndarray: euler angle representation of the EE orientation (roll, pitch, yaw with
                static reference frame) (shape: [3,])
        """
        if (arm != 'right' and arm != 'left'):
            print('arm must be either "right" or "left"')
            print('defaulting to "right"')
            arm = 'right'
        if arm == 'left':
            ee_link_id = self.ee_link_l_id
        else:
            ee_link_id = self.ee_link_r_id
        info = p.getLinkState(self.robot_id, ee_link_id)
        pos = info[4]
        quat = info[5]

        rot_mat = arutil.quat2rot(quat)
        euler = arutil.quat2euler(quat, axes='xyz')  # [roll, pitch, yaw]
        return np.array(pos), np.array(quat), rot_mat, euler

    def get_ee_force(self):
        """
        TODO add force sensor on the end effector
        """
        raise NotImplementedError

    def setup_camera(self, focus_pt=None, dist=3, yaw=0, pitch=0, roll=0):
        """
        Setup the camera view matrix and projection matrix. Must be called
        first before images are renderred

        Args:
            focus_pt (list): position of the target (focus) point,
                in Cartesian world coordinates
            dist (float): distance from eye (camera) to the focus point
            yaw (float): yaw angle in degrees,
                left/right around up-axis (z-axis).
            pitch (float): pitch in degrees, up/down.
            roll (float): roll in degrees around forward vector
        """
        self.camera.setup_camera(focus_pt=focus_pt, dist=dist,
                                 yaw=yaw, pitch=pitch, roll=roll)

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgba/depth images

        Args:
            get_rgb (bool): return rgb image if True, None otherwise
            get_depth (bool): return depth image if True, None otherwise

        Returns:
            rgba and depth images (np.ndarray)
        """
        return self.camera.get_images(get_rgb, get_depth)

    def compute_ik(self, pos, ori=None, arm='right', nullspace=True,
                   *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector

        Args:
            pos (list or np.ndarray): position (shape: [3,])
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: [3,]), or
                quaternion ([qx, qy, qz, qw], shape: [4,]),
                or rotation matrix (shape: [3, 3]).

        Returns:
            list: inverse kinematics solution (shape: [14,])
        """
        if (arm != 'right' and arm != 'left'):
            print('arm must be either "right" or "left"')
            print('defaulting to "right"')
            arm = 'right'
        if arm == 'left':
            ee_link_id = self.ee_link_l_id
        else:
            ee_link_id = self.ee_link_r_id
        if ori is not None:
            ori = np.array(ori)
            if ori.size == 3:
                # [roll, pitch, yaw]
                ori = arutil.euler2quat(ori, axes='xyz')
            elif ori.shape == (3, 3):
                ori = arutil.rot2quat(ori)
            elif ori.size != 4:
                raise ValueError('Orientation should be rotation matrix, '
                                 'euler angles or quaternion')
            if nullspace:
                ll, ul, jr, rp = self._get_joint_ranges()
                jnt_poss = p.calculateInverseKinematics(
                    self.robot_id,
                    ee_link_id,
                    pos,
                    ori,
                    jointDamping=self._ik_jds,
                    lowerLimits=ll,
                    upperLimits=ul,
                    jointRanges=jr,
                    restPoses=rp)
            else:
                jnt_poss = p.calculateInverseKinematics(
                    self.robot_id,
                    ee_link_id,
                    pos,
                    ori,
                    jointDamping=self._ik_jds)
        else:
            if nullspace:
                ll, ul, jr, rp = self._get_joint_ranges()
                jnt_poss = p.calculateInverseKinematics(
                    self.robot_id,
                    ee_link_id,
                    pos,
                    jointDamping=self._ik_jds,
                    lowerLimits=ll,
                    upperLimits=ul,
                    jointRanges=jr,
                    restPoses=rp
                    )
            else:
                jnt_poss = p.calculateInverseKinematics(
                    self.robot_id,
                    ee_link_id,
                    pos,
                    jointDamping=self._ik_jds)
        jnt_poss = list(jnt_poss)

        return jnt_poss

    def _get_joint_ranges(self):
        """
        Computes the optional parameters needed to compute null-space
        inverse kinematics

        Returns:
            lower_limits (list): Lower joint limits, per joint
            upper_limits (list): upper joint limits, per joint
            joint_ranges (list): joint range, per joint
            rest_poses (list): poses to try to stay near for IK solution
        """
        lower_limits = []
        upper_limits = []
        joint_ranges = []
        rest_poses = []

        current_positions = p.getJointStates(self.robot_id, self.arm_jnt_ids)

        for i in range(len(self.arm_jnt_ids)):
            # rest_pose = p.getJointState(self.robot_id, i)[0]
            lower_limits.append(-2.8)
            upper_limits.append(2.8)
            joint_ranges.append(5.4)
            rest_poses.append(current_positions[i])

        return lower_limits, upper_limits, joint_ranges, rest_poses

    def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Block the code to wait for the joint moving to the specified goal.
        The goal can be a desired velocity(s) or a desired position(s).
        Max waiting time is self.cfgs.TIMEOUT_LIMIT

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the actuated
                joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pos' or 'vel'

        Returns:
            if the goal is reached or not
        """
        success = False
        start_time = time.time()
        while True:
            if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                pt_str = 'Unable to move to joint goals [mode: %s] (%s)' \
                         ' within %f s' % (mode, str(goal),
                                           self.cfgs.TIMEOUT_LIMIT)
                arutil.print_red(pt_str)
                return success
            if self._reach_jnt_goal(goal, joint_name, mode=mode):
                success = True
                break
            time.sleep(0.001)
        return success

    def _reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Check if the joint reached the goal or not.
        The goal can be a desired velocity(s) or a desired position(s).

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the
                actuated joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pose' or 'vel'

        Returns:
            if the goal is reached or not
        """
        goal = np.array(goal)
        if mode == 'pos':
            new_jnt_val = self.get_jpos(joint_name)
        elif mode == 'vel':
            new_jnt_val = self.get_jvel(joint_name)
        else:
            raise ValueError('Only pos and vel modes are supported!')
        new_jnt_val = np.array(new_jnt_val)
        jnt_diff = new_jnt_val - goal
        error = np.max(np.abs(jnt_diff))
        if error < self.cfgs.MAX_JOINT_ERROR:
            return True
        else:
            return False

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
        self._step_sim_mode = False
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
        self._home_position = self.cfgs.HOME_POSITION
        # joint damping for inverse kinematics
        self._ik_jd = 0.05
        self._thread_sleep = 0.001
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.arm_jnt_names = [
            'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
            'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
            'yumi_joint_6_r',
            'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
            'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
            'yumi_joint_6_l'
        ]

        self.ee_link_jnt_r = self.cfgs.ROBOT_EE_FRAME_JOINT_RIGHT
        self.ee_link_jnt_l = self.cfgs.ROBOT_EE_FRAME_JOINT_LEFT

        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_dof = len(self.arm_jnt_names)

        self.rvl_joint_names = self.arm_jnt_names
        self._ik_jds = [self._ik_jd] * len(self.rvl_joint_names)

        max_torques = [140]*3 + [20]*4
        self._max_torques =  max_torques + max_torques
        self.camera = PyBulletCamera(p, self.cfgs)

    def _rt_simulation(self):
        """
        Run step simulation all the time in backend
        """
        while True:
            if not self._step_sim_mode:
                p.stepSimulation()
            time.sleep(self._thread_sleep)

    def _build_jnt_id(self):
        """
        Build the mapping from the joint name to joint index
        """
        self.jnt_to_id = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            jnt_name = info[1].decode('UTF-8')
            self.jnt_to_id[jnt_name] = info[0]

        # # joint ids for the actuators
        # # only the first joint in the gripper is the actuator
        # # in the simulation
        # act_joint_names = self.arm_jnt_names + [self.gripper_jnt_names[0]]
        # self.actuator_ids = [self.jnt_to_id[jnt] for jnt in act_joint_names]

        # # joint ids for all joints
        # self.rvl_jnt_ids = [
        #     self.jnt_to_id[jnt] for jnt in self.rvl_joint_names
        # ]

        self.ee_link_r_id = self.jnt_to_id[self.ee_link_jnt_r]
        self.ee_link_l_id = self.jnt_to_id[self.ee_link_jnt_l]

        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]

