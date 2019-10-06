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


class UR5eRobotPybullet(Robot):
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
        super(UR5eRobotPybullet, self).__init__(cfgs=cfgs)
        self._render = render
        self.self_collision = self_collision
        self.p = p
        if self._render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
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
        plane_ori = arutil.quat2euler([0, 0, 0])
        self.plane_id = p.loadURDF("plane.urdf", plane_pos, plane_ori)

        ur_pos = [0, 0, 1]
        ur_ori = arutil.quat2euler([0, 0, 0])
        if self.self_collision:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF,
                                       ur_pos,
                                       ur_ori,
                                       flags=p.URDF_USE_SELF_COLLISION)
        else:
            self.robot_id = p.loadURDF(self.cfgs.PYBULLET_URDF, ur_pos, ur_ori)
        self._build_jnt_id()
        if self.self_collision:
            # weird behavior occurs on the gripper
            # when self-collision is enforced
            self._disable_gripper_self_collision()

    def _disable_gripper_self_collision(self):
        for i in range(len(self.gripper_jnt_names)):
            for j in range(i + 1, len(self.gripper_jnt_names)):
                jnt_idx1 = self.jnt_to_id[self.gripper_jnt_names[i]]
                jnt_idx2 = self.jnt_to_id[self.gripper_jnt_names[j]]
                p.setCollisionFilterPair(self.robot_id,
                                         self.robot_id,
                                         jnt_idx1,
                                         jnt_idx2,
                                         enableCollision=0)

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

    def open_gripper(self):
        """
        Open the gripper

        Returns:
            return if the action is sucessful or not
        """
        success = self._set_gripper_pos(self.gripper_open_angle)
        return success

    def close_gripper(self):
        """
        Close the gripper

        Returns:
            return if the action is sucessful or not
        """
        success = self._set_gripper_pos(self.gripper_close_angle)
        return success

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
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
            if len(position) != 6:
                raise ValueError('Position should contain 6'
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

    def set_jvel(self, velocity, joint_name=None, wait=False, *args, **kwargs):
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
            if len(velocity) != 6:
                raise ValueError('Velocity should contain 6 elements '
                                 'if the joint_name is not provided')
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
            if len(torque) != 6:
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

    def set_ee_pose(self, pos, ori=None, wait=True, *args, **kwargs):
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
        jnt_pos = self.compute_ik(pos, ori)
        success = self.set_jpos(jnt_pos, wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
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
        if self._step_sim_mode:
            raise AssertionError('move_ee_xyz() can '
                                 'only be called in realtime'
                                 ' simulation mode')
        pos, quat, rot_mat, euler = self.get_ee_pose()
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
                                           quat)
            way_jnt_positions.append(copy.deepcopy(tgt_jnt_poss))
        success = False
        for jnt_poss in way_jnt_positions:
            success = self.set_jpos(jnt_poss)
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

    def get_ee_pose(self):
        """
        Return the end effector pose

        Returns:
            np.ndarray: x, y, z position of the EE (shape: [3,])
            np.ndarray: quaternion representation of the EE orientation (shape: [4,])
            np.ndarray: rotation matrix representation of the EE orientation (shape: [3, 3])
            np.ndarray: euler angle representation of the EE orientation (roll, pitch, yaw with
                static reference frame) (shape: [3,])
        """
        info = p.getLinkState(self.robot_id, self.ee_link_id)
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

    def compute_ik(self, pos, ori=None, *args, **kwargs):
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
            inverse kinematics solution (joint angles, list)
        """
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
            jnt_poss = p.calculateInverseKinematics(self.robot_id,
                                                    self.ee_link_id,
                                                    pos,
                                                    ori,
                                                    jointDamping=self._ik_jds)
        else:
            jnt_poss = p.calculateInverseKinematics(self.robot_id,
                                                    self.ee_link_id,
                                                    pos,
                                                    jointDamping=self._ik_jds)
        jnt_poss = list(jnt_poss)
        return jnt_poss[:len(self.arm_jnt_ids)]

    def _mimic_gripper(self, joint_val):
        """
        Given the value for the first joint,
        mimic the joint values for the rest joints
        """
        jnt_vals = [joint_val]
        for i in range(1, len(self.gripper_jnt_names)):
            jnt_vals.append(joint_val * self._gripper_mimic_coeff[i])
        return jnt_vals

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

        # gripper thread
        self._th_gripper = threading.Thread(target=self._th_mimic_gripper)
        self._th_gripper.daemon = True
        self._th_gripper.start()

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
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_dof = len(self.arm_jnt_names)
        self._gripper_mimic_coeff = [1, -1, 1, -1, -1, 1]
        self.gripper_jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self.gripper_close_angle = 0.7
        self.gripper_open_angle = 0
        self.gripper_jnt_names_set = set(self.gripper_jnt_names)
        self.rvl_joint_names = self.arm_jnt_names + self.gripper_jnt_names
        self._ik_jds = [self._ik_jd] * len(self.rvl_joint_names)
        self.ee_link_jnt = self.cfgs.ROBOT_EE_FRAME_JOINT

        # https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
        self._max_torques = [150, 150, 150, 28, 28, 28]
        # a random value for robotiq joints
        self._max_torques.append(20)
        self.camera = PyBulletCamera(p, self.cfgs)

    def _rt_simulation(self):
        """
        Run step simulation all the time in backend
        """
        while True:
            if not self._step_sim_mode:
                p.stepSimulation()
            time.sleep(self._thread_sleep)

    def _th_mimic_gripper(self):
        """
        Make all the other joints of the gripper
        follow the motion of the first joint of the gripper
        """
        while True:
            max_torq = self._max_torques[-1]
            max_torques = [max_torq] * (len(self.gripper_jnt_names) - 1)
            gripper_pos = self.get_jpos(self.gripper_jnt_names[0])
            gripper_poss = self._mimic_gripper(gripper_pos)
            p.setJointMotorControlArray(self.robot_id,
                                        self.gripper_jnt_ids[1:],
                                        p.POSITION_CONTROL,
                                        targetPositions=gripper_poss[1:],
                                        forces=max_torques)
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

        self.ee_link_id = self.jnt_to_id[self.ee_link_jnt]
        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]
        self.gripper_jnt_ids = [
            self.jnt_to_id[jnt] for jnt in self.gripper_jnt_names
        ]

    def _set_gripper_pos(self, position):
        """
        Set the gripper position. We make a separate function apart from
        set_jpos to make the api consistent with the real robot. set_jpos
        is only used to control the robot arm

        Args:
            position (float): joint position

        Returns:
            A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        joint_name = self.gripper_jnt_names[0]
        gripper_pos = position[-1]
        tgt_pos = arutil.clamp(gripper_pos,
                        self.gripper_open_angle,
                        self.gripper_close_angle)
        max_torque = self._max_torques[-1]
        jnt_id = self.jnt_to_id[joint_name]
        p.setJointMotorControl2(self.robot_id,
                                jnt_id,
                                p.POSITION_CONTROL,
                                targetPosition=tgt_pos,
                                force=max_torque)
        success = False
        if not self._step_sim_mode and wait:
            success = self._wait_to_reach_jnt_goal(tgt_pos,
                                                   joint_name=joint_name,
                                                   mode='pos')
        return success
