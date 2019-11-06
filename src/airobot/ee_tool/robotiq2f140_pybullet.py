import threading
import time

import airobot.utils.common as arutil
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
        self.p = PB_CLIENT
        self._gripper_mimic_coeff = [1, -1, 1, -1, -1, 1]
        self.jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self._step_sim_mode = False
        self._thread_sleep = 0.001
        self.max_torque = 10.0
        self.gripper_close_angle = self.cfgs.EETOOL.CLOSE_ANGLE
        self.gripper_open_angle = self.cfgs.EETOOL.OPEN_ANGLE
        self.jnt_names_set = set(self.jnt_names)
        self._gripper_activated = False

    def activate(self, robot_id, jnt_to_id):
        """
        Setup the gripper

        Args:
            robot_id (int): robot id in Pybullet
            jnt_to_id (dict): mapping from the joint name to joint id

        """
        if not self._gripper_activated:
            self.robot_id = robot_id
            self.jnt_to_id = jnt_to_id
            self.gripper_jnt_ids = [
                self.jnt_to_id[jnt] for jnt in self.jnt_names
            ]
            self._gripper_activated = True
            # gripper thread
            self._th_gripper = threading.Thread(target=self._th_mimic_gripper)
            self._th_gripper.daemon = True
            self._th_gripper.start()
        else:
            return

    def open(self):
        """
        Open the gripper

        Returns:
            bool: return if the action is sucessful or not
        """
        if not self._gripper_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_pos(self.gripper_open_angle)
        return success

    def close(self):
        """
        Close the gripper

        Returns:
            bool: return if the action is sucessful or not
        """
        if not self._gripper_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_pos(self.gripper_close_angle)
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
                                     force=self.max_torque)
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
        if not self._gripper_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        pos = self.p.getJointState(self.robot_id, jnt_id)[0]
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
        if not self._gripper_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        vel = self.p.getJointState(self.robot_id, jnt_id)[1]
        return vel

    def disable_gripper_self_collision(self):
        """
        Disable the gripper collision checking in Pybullet
        """
        if not self._gripper_activated:
            raise RuntimeError('Call activate function first!')
        for i in range(len(self.jnt_names)):
            for j in range(i + 1, len(self.jnt_names)):
                jnt_idx1 = self.jnt_to_id[self.jnt_names[i]]
                jnt_idx2 = self.jnt_to_id[self.jnt_names[j]]
                self.p.setCollisionFilterPair(self.robot_id,
                                              self.robot_id,
                                              jnt_idx1,
                                              jnt_idx2,
                                              enableCollision=0)

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
            max_torq = self.max_torque
            max_torques = [max_torq] * (len(self.jnt_names) - 1)
            gripper_pos = self.get_pos()
            gripper_poss = self._mimic_gripper(gripper_pos)
            self.p.setJointMotorControlArray(self.robot_id,
                                             self.gripper_jnt_ids[1:],
                                             self.p.POSITION_CONTROL,
                                             targetPositions=gripper_poss[1:],
                                             forces=max_torques)
            time.sleep(self._thread_sleep)
