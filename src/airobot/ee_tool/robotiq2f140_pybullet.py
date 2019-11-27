import threading
import time

import airobot.utils.common as arutil
import pybullet as p
from airobot.ee_tool.ee import EndEffectorTool
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from airobot.utils.pb_util import PB_CLIENT


class Robotiq2F140Pybullet(EndEffectorTool):
    """
    Class for interfacing with a Robotiq 2F140 gripper when
    it is attached to UR5e arm in pybullet.
    """

    def __init__(self, cfgs):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the gripper
        """
        super(Robotiq2F140Pybullet, self).__init__(cfgs=cfgs)
        self.p = p
        self._gripper_mimic_coeff = [1, -1, 1, -1, -1, 1]
        self.jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint',
        ]

        self._step_sim_mode = False
        self.max_torque = 10.0
        self.gripper_close_angle = self.cfgs.EETOOL.CLOSE_ANGLE
        self.gripper_open_angle = self.cfgs.EETOOL.OPEN_ANGLE
        self.jnt_names_set = set(self.jnt_names)
        self._mthread_started = False
        self.deactivate()

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper

        Args:
            robot_id (int): robot id in Pybullet
            jnt_to_id (dict): mapping from the joint name to joint id

        """
        self.robot_id = robot_id
        self.jnt_to_id = jnt_to_id
        self.gripper_jnt_ids = [
            self.jnt_to_id[jnt] for jnt in self.jnt_names
        ]
        p.changeDynamics(self.robot_id,
                         self.jnt_to_id['left_inner_finger_pad_joint'],
                         lateralFriction=2.0,
                         spinningFriction=1.0,
                         rollingFriction=1.0)
        p.changeDynamics(self.robot_id,
                         self.jnt_to_id['right_inner_finger_pad_joint'],
                         lateralFriction=2.0,
                         spinningFriction=1.0,
                         rollingFriction=1.0)
        # if the gripper has been activated once,
        # the following code is used to prevent starting
        # a new thread after the arm reset if a thread has been started

        if not self._mthread_started:
            self._mthread_started = True
            # gripper thread
            self._th_gripper = threading.Thread(target=self._th_mimic_gripper)
            self._th_gripper.daemon = True
            self._th_gripper.start()
        else:
            return

    def open(self, wait=True):
        """
        Open the gripper

        Returns:
            bool: return if the action is sucessful or not
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_pos(self.gripper_open_angle,
                               wait=wait)
        return success

    def close(self, wait=True):
        """
        Close the gripper

        Returns:
            bool: return if the action is sucessful or not
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_pos(self.gripper_close_angle,
                               wait=wait)
        return success

    def set_pos(self, pos, wait=True):
        """
        Set the gripper position.

        Args:
            pos (float): joint position
            wait (bool): wait until the joint position is set
                to the target position

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits
        """
        joint_name = self.jnt_names[0]
        tgt_pos = arutil.clamp(pos,
                               self.gripper_open_angle,
                               self.gripper_close_angle)
        jnt_id = self.jnt_to_id[joint_name]
        self.p.setJointMotorControl2(self.robot_id,
                                     jnt_id,
                                     self.p.POSITION_CONTROL,
                                     targetPosition=tgt_pos,
                                     force=self.max_torque,
                                     physicsClientId=PB_CLIENT)
        success = False
        if not self._step_sim_mode and wait:
            success = wait_to_reach_jnt_goal(
                tgt_pos,
                get_func=self.get_pos,
                joint_name=joint_name,
                get_func_derv=self.get_vel,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_ERROR
            )
        return success

    def get_pos(self, joint_name=None):
        """
        Return the joint position(s) of the gripper.
        Joint name is not required, we add this here just to
        make the api consistent. Also, it's used in
        function `wait_to_reach_jnt_goal`

        Returns:
            float: joint position
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        pos = self.p.getJointState(self.robot_id, jnt_id,
                                   physicsClientId=PB_CLIENT)[0]
        return pos

    def get_vel(self, joint_name=None):
        """
        Return the joint velocity of the gripper.
        Joint name is not required, we add this here just to
        make the api consistent. Also, it's used in
        function `wait_to_reach_jnt_goal`

        Returns:
            float: joint velocity
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        vel = self.p.getJointState(self.robot_id, jnt_id,
                                   physicsClientId=PB_CLIENT)[1]
        return vel

    def disable_gripper_self_collision(self):
        """
        Disable the gripper collision checking in Pybullet
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        for i in range(len(self.jnt_names)):
            for j in range(i + 1, len(self.jnt_names)):
                jnt_idx1 = self.jnt_to_id[self.jnt_names[i]]
                jnt_idx2 = self.jnt_to_id[self.jnt_names[j]]
                self.p.setCollisionFilterPair(self.robot_id,
                                              self.robot_id,
                                              jnt_idx1,
                                              jnt_idx2,
                                              enableCollision=0,
                                              physicsClientId=PB_CLIENT)

    def _mimic_gripper(self, joint_val):
        """
        Given the value for the first joint,
        mimic the joint values for the rest joints
        """
        jnt_vals = [joint_val]
        for i in range(1, len(self.jnt_names)):
            jnt_vals.append(joint_val * self._gripper_mimic_coeff[i])
        return jnt_vals

    def _th_mimic_gripper(self):
        """
        Make all the other joints of the gripper
        follow the motion of the first joint of the gripper
        """
        while True:
            if self._is_activated:
                max_torq = self.max_torque
                max_torques = [max_torq] * (len(self.jnt_names) - 1)
                gripper_pos = self.get_pos()
                gripper_poss = self._mimic_gripper(gripper_pos)[1:]
                gripper_vels = [0.0] * len(max_torques)
                self.p.setJointMotorControlArray(self.robot_id,
                                                 self.gripper_jnt_ids[1:],
                                                 self.p.POSITION_CONTROL,
                                                 targetPositions=gripper_poss,
                                                 targetVelocities=gripper_vels,
                                                 forces=max_torques,
                                                 physicsClientId=PB_CLIENT)
            time.sleep(0.05)

    def deactivate(self):
        self._is_activated = False

    def activate(self):
        self._is_activated = True
