from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numpy as np
import pybullet as p
import pybullet_data
import threading
import time
from gym.utils import seeding
from airobot import arutil
from airobot.robot.robot import Robot


class UR5eRobotPybullet(Robot):
    def __init__(self, cfgs, render=False,
                 seed=None, self_collision=False):
        super(UR5eRobotPybullet, self).__init__(cfgs=cfgs)
        self._render = render
        self.self_collision = self_collision
        self.p = p
        if self._render:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        self.np_random, _ = self._seed(seed)
        self._init_consts()
        self.reset()
        self._init_threads()

    def __del__(self):
        p.disconnect()

    def _seed(self, seed=None):
        np_random, seed = seeding.np_random(seed)
        return np_random, seed

    def _init_threads(self):
        # realtime simulation thread
        self._step_sim_mode = False
        if not self._render:
            self._th_sim = threading.Thread(target=self._th_realtime_simulation)
            self._th_sim.daemon = True
            self._th_sim.start()
        else:
            self.realtime_simulation(True)

        # gripper thread
        self._th_gripper = threading.Thread(target=self._th_mimic_gripper)
        self._th_gripper.daemon = True
        self._th_gripper.start()

    def _init_consts(self):
        # joint damping for inverse kinematics
        self.ik_jd = 0.05
        self._thread_sleep = 0.001
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.arm_jnt_names = ['shoulder_pan_joint',
                              'shoulder_lift_joint',
                              'elbow_joint',
                              'wrist_1_joint',
                              'wrist_2_joint',
                              'wrist_3_joint']

        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.gripper_mimic = [1, -1, 1, -1, -1, 1]
        self.gripper_jnt_names = ['finger_joint',
                                  'left_inner_knuckle_joint',
                                  'left_inner_finger_joint',
                                  'right_outer_knuckle_joint',
                                  'right_inner_knuckle_joint',
                                  'right_inner_finger_joint']
        self.gripper_close_angle = 0.7
        self.gripper_open_angle = 0
        self.gripper_jnt_names_set = set(self.gripper_jnt_names)
        self.rvl_joint_names = self.arm_jnt_names + self.gripper_jnt_names
        self.ik_jds = [self.ik_jd] * len(self.rvl_joint_names)
        self.ee_link = 'wrist_3_link-tool0_fixed_joint'

        # https://www.universal-robots.com/how-tos-and-faqs/
        # faq/ur-faq/max-joint-torques-17260/
        self.max_torques = [150, 150, 150, 28, 28, 28]
        # some random values for robotiq joints
        self.max_torques.append(20)

    def _th_realtime_simulation(self):
        while True:
            if not self._step_sim_mode:
                p.stepSimulation()
            time.sleep(self._thread_sleep)

    def _th_mimic_gripper(self):
        while True:
            max_torq = self.max_torques[-1]
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
        self.jnt_to_id = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            jnt_name = info[1].decode('UTF-8')
            self.jnt_to_id[jnt_name] = info[0]

        # joint ids for the actuators
        # only the first joint in the gripper is the actuator
        # in the simulation
        act_joint_names = self.arm_jnt_names + [self.gripper_jnt_names[0]]
        self.actuator_ids = [self.jnt_to_id[jnt]
                             for jnt in act_joint_names]
        # joint ids for all joints
        self.rvl_jnt_ids = [self.jnt_to_id[jnt]
                            for jnt in self.rvl_joint_names]

        self.ee_link_id = self.jnt_to_id[self.ee_link]
        self.arm_jnt_ids = [self.jnt_to_id[jnt]
                            for jnt in self.arm_jnt_names]
        self.gripper_jnt_ids = [self.jnt_to_id[jnt]
                                for jnt in self.gripper_jnt_names]

    def go_home(self):
        # pre-defined home pose
        # 6 joints for the arm, 7th joint for the gripper
        jnt_positions = [0., -1.5, 2.0, -2.05, -1.57, 0]
        # jnt_positions = [0., -1.1, 2.2, -2.65, -1.57, 0]
        self.set_jposs(jnt_positions)

    def reset(self):
        p.resetSimulation()

        plane_pos = [0, 0, 0]
        plane_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.plane_id = p.loadURDF("plane.urdf",
                                   plane_pos,
                                   plane_ori)

        ur_pos = [0, 0, 1]
        ur_ori = p.getQuaternionFromEuler([0, 0, 0])
        if self.self_collision:
            # weird behavior occurs on the gripper
            # when self-collision is enforced
            self.robot_id = p.loadURDF(self.cfgs.URDF,
                                       ur_pos,
                                       ur_ori,
                                       flags=p.URDF_USE_SELF_COLLISION)
        else:
            self.robot_id = p.loadURDF(self.cfgs.URDF,
                                       ur_pos,
                                       ur_ori)
        self._build_jnt_id()

    def step_simulation(self):
        p.stepSimulation()

    def realtime_simulation(self, on=True):
        if on:
            self._step_sim_mode = False
            if self._render:
                p.setRealTimeSimulation(1)
        else:
            self._step_sim_mode = True
            if self._render:
                p.setRealTimeSimulation(0)

    def open_gripper(self):
        self.set_jpos(self.gripper_jnt_names[0], self.gripper_open_angle)

    def close_gripper(self):
        self.set_jpos(self.gripper_jnt_names[0], self.gripper_close_angle)

    def set_jpos(self, joint_name, position):
        jnt_id = self._jnt_id_from_name(joint_name)
        if joint_name in self.gripper_jnt_names:
            mimic_factor = self.gripper_mimic[self.gripper_jnt_names.index(joint_name)]
            finger_jnt_pos = position / float(mimic_factor)
            finger_jnt_pos = max(self.gripper_open_angle, finger_jnt_pos)
            finger_jnt_pos = min(self.gripper_close_angle, finger_jnt_pos)
            gripper_poss = self._mimic_gripper(finger_jnt_pos)
            p.setJointMotorControlArray(self.robot_id,
                                        self.gripper_jnt_ids,
                                        p.POSITION_CONTROL,
                                        targetPositions=gripper_poss,
                                        forces=self.max_torques[-len(self.gripper_jnt_ids):])
        else:
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.POSITION_CONTROL,
                                    targetPosition=position,
                                    force=self.max_torques[self.rvl_joint_names.index(joint_name)])

    def set_jposs(self, positions):
        """
        set joint positions

        Joint angle range for the gripper is [0, 0.7].
        ([self.gripper_open_angle, self.gripper_close_angle])
        0 is fully open, 0.7 is fully closed

        :param positions: joint positions, if length
                          is 6, pad the current gripper
                          joint position in the end to make
                          the length to be 7
        """
        positions = copy.deepcopy(positions)
        if len(positions) == 6:
            # copy the current gripper joint position
            gripper_pos = self.get_jpos(self.gripper_jnt_names[0])
            positions.append(gripper_pos)
        assert len(positions) == 7
        gripper_pos = positions[-1]
        gripper_pos = max(self.gripper_open_angle, gripper_pos)
        gripper_pos = min(self.gripper_close_angle, gripper_pos)
        positions[-1] = gripper_pos
        p.setJointMotorControlArray(self.robot_id,
                                    self.actuator_ids,
                                    p.POSITION_CONTROL,
                                    targetPositions=positions,
                                    forces=self.max_torques)

    def set_jvel(self, joint_name, velocity):
        jnt_id = self._jnt_id_from_name(joint_name)
        p.setJointMotorControl2(self.robot_id,
                                jnt_id,
                                p.VELOCITY_CONTROL,
                                targetVelocity=velocity,
                                force=self.max_torques[self.rvl_joint_names.index(joint_name)])

    def set_jvels(self, velocities):
        """
        set joint velocities

        :param velocities: joint velocities, if length
                          is 6, pad the current gripper
                          joint velocity in the end to make
                          the length to be 7
        """
        velocities = copy.deepcopy(velocities)
        if len(velocities) == 6:
            # copy the current gripper joint position
            gripper_vel = self.get_jvel(self.gripper_jnt_names[0])
            velocities.append(gripper_vel)
        assert len(velocities) == 7
        p.setJointMotorControlArray(self.robot_id,
                                    self.actuator_ids,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=velocities,
                                    forces=self.max_torques)

    def set_jtorq(self, joint_name, torque):
        """
        Apply torque to the specified joint,
        call enable_torque_control(joint_name)
        before doing torque control
        Note: call to this function is only effective in this simulation step.
              you need to supply torque value for each
              simulation step to do the torque control
        :param joint_name: which joint to apply the torque onto
        :param torque: the torque to apply in this simulation step
        :type joint_name: str
        :type torque: float
        """
        jnt_id = self._jnt_id_from_name(joint_name)

        p.setJointMotorControl2(self.robot_id,
                                jnt_id,
                                p.TORQUE_CONTROL,
                                force=torque)

    def set_jtorqs(self, torques):
        """
        set joint torques, call enable_torque_control() before doing
        torque control

        You need to provide torque values for each simulation
        step to do the torque control. It's easier to use torque control
        in step_simulation mode instead of realtime_simulation mode.
        If you are using realtime_simulation mode, the time interval
        between two set_jtorqs() calls must be small enough (like 0.0002s)

        :param torques: joint torques, only support ur arm for now (6 joints)
        :type torques: list
        """
        torques = copy.deepcopy(torques)
        assert len(torques) == 6
        p.setJointMotorControlArray(self.robot_id,
                                    self.actuator_ids[:-1],
                                    p.TORQUE_CONTROL,
                                    forces=torques)

    def set_ee_pose(self, pos, ori=None):
        jnt_poss = self.compute_ik(pos, ori)
        self.set_jposs(jnt_poss)
        return jnt_poss

    def move_ee_xyz(self, delta_xyz, eef_step=0.005):
        assert not self._step_sim_mode, \
            'move_ee_xyz() can only be called in realtime simulation mode'
        success = True
        pos, quat, rot_mat, euler = self.get_ee_pose()
        cur_pos = np.array(pos)
        delta_xyz = np.array(delta_xyz)
        path_len = np.linalg.norm(delta_xyz)
        num_pts = int(np.ceil(path_len / float(eef_step)))
        if num_pts <= 1:
            num_pts = 2
        waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
        waypoints = cur_pos + waypoints_sp / float(path_len) * delta_xyz
        for i in range(waypoints.shape[0]):
            tgt_jnt_poss = self.set_ee_pose(waypoints[i, :].flatten().tolist(),
                                            quat)
            tgt_jnt_poss = np.array(tgt_jnt_poss)
            start_time = time.time()
            while True:
                if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                    pt_str = 'Unable to move to joint angles (%s)' \
                             ' within %f s' % (str(tgt_jnt_poss.tolist()),
                                               self.cfgs.TIMEOUT_LIMIT)
                    arutil.print_red(pt_str)
                    success = False
                    return success
                new_jnt_poss = self.get_jposs()
                new_jnt_poss = np.array(new_jnt_poss)
                jnt_diff = new_jnt_poss - tgt_jnt_poss
                error = np.max(np.abs(jnt_diff))
                if error < self.cfgs.MAX_JOINT_ERROR:
                    break
                time.sleep(0.001)
        return success

    def enable_torque_control(self, joint_name=None):
        if joint_name is None:
            tgt_vels = [0.0] * len(self.actuator_ids[:-1])
            forces = [0.0] * len(self.actuator_ids[:-1])
            p.setJointMotorControlArray(self.robot_id,
                                        self.actuator_ids[:-1],
                                        p.VELOCITY_CONTROL,
                                        targetVelocities=tgt_vels,
                                        forces=forces)
        else:
            jnt_id = self._jnt_id_from_name(joint_name)
            p.setJointMotorControl2(self.robot_id,
                                    jnt_id,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0.0)

    def disable_torque_control(self, joint_name=None):
        if joint_name is None:
            self.set_jvels([0.0] * 7)
        else:
            self.set_jvel(joint_name, 0.0)

    def _jnt_id_from_name(self, joint_name):
        if joint_name in self.rvl_joint_names:
            jnt_id = self.jnt_to_id[joint_name]
        else:
            raise ValueError('unknown movable joint'
                             ' name: %s' % joint_name)
        return jnt_id

    def get_jposs(self):
        states = p.getJointStates(self.robot_id,
                                  self.actuator_ids)
        pos = [state[0] for state in states]
        return pos

    def get_jpos(self, joint_name):
        jnt_id = self._jnt_id_from_name(joint_name)
        pos = p.getJointState(self.robot_id, jnt_id)[0]
        return pos

    def get_jvels(self):
        states = p.getJointStates(self.robot_id,
                                  self.actuator_ids)
        vel = [state[1] for state in states]
        return vel

    def get_jvel(self, joint_name):
        jnt_id = self._jnt_id_from_name(joint_name)
        vel = p.getJointState(self.robot_id, jnt_id)[1]
        return vel

    def get_jtorqs(self):
        """
        Get joint torque applied during the last stepSimulation
        in VELOCITY_CONTROL and POSITION_CONTROL modes
        In TORQUE_CONTROL, the applied joint motor torque is
        exactly what you provide, so there is no need to report it separately.
        """
        states = p.getJointStates(self.robot_id,
                                  self.actuator_ids)
        # state[3] is appliedJointMotorTorque
        torques = [state[3] for state in states]
        return torques

    def get_jtorq(self, joint_name):
        jnt_id = self._jnt_id_from_name(joint_name)
        torque = p.getJointState(self.robot_id, jnt_id)[3]
        return torque

    def get_ee_pose(self):
        info = p.getLinkState(self.robot_id, self.ee_link_id)
        pos = info[4]
        quat = info[5]
        rot_mat = p.getMatrixFromQuaternion(quat)
        euler = p.getEulerFromQuaternion(quat)  # [roll, pitch, yaw]
        return list(pos), list(quat), list(rot_mat), list(euler)

    def get_ee_force(self):
        """
        TODO add force sensor on the end effector
        """
        raise NotImplementedError

    def compute_ik(self, pos, ori=None):
        if ori is not None:
            if len(ori) == 3:
                # [roll, pitch, yaw]
                ori = p.getQuaternionFromEuler(ori)
            assert len(ori) == 4
            jnt_poss = p.calculateInverseKinematics(self.robot_id,
                                                    self.ee_link_id,
                                                    pos,
                                                    ori,
                                                    jointDamping=self.ik_jds)
        else:
            jnt_poss = p.calculateInverseKinematics(self.robot_id,
                                                    self.ee_link_id,
                                                    pos,
                                                    jointDamping=self.ik_jds)
        jnt_poss = list(jnt_poss)
        return jnt_poss[:len(self.actuator_ids)]

    def _mimic_gripper(self, joint_val):
        """
        Given the value for the first joint,
        mimic the joint values for the rest joints
        """
        jnt_vals = [joint_val]
        for i in range(1, len(self.gripper_jnt_names)):
            jnt_vals.append(joint_val * self.gripper_mimic[i])
        return jnt_vals
