import threading
import time

import airobot.utils.common as arutil
from airobot.ee_tool.simple_gripper_pybullet import SimpleGripperPybullet
from airobot.utils.arm_util import wait_to_reach_jnt_goal


class SimpleGripperMimicPybullet(SimpleGripperPybullet):
    """
    A base class for gripper with mimic joints in pybullet.

    Args:
        cfgs (YACS CfgNode): configurations for the gripper.
        pb_client (BulletClient): pybullet client.

    Attributes:
        cfgs (YACS CfgNode): configurations for the gripper.
        gripper_close_angle (float): position value corresponding to the
            fully closed position of the gripper.
        gripper_open_angle (float): position value corresponding to the
            fully open position of the gripper.
        jnt_names (list): names of the gripper joints.
        gripper_jnt_ids (list): pybullet joint ids of the gripper joints.
        robot_id (int): robot id in Pybullet.
        jnt_to_id (dict): mapping from the joint name to joint id.
    """

    def __init__(self, cfgs, pb_client):
        super(SimpleGripperMimicPybullet, self).__init__(cfgs=cfgs, pb_client=pb_client)
        self._gripper_mimic_coeff = self.cfgs.EETOOL.MIMIC_COEFF
        self._mthread_started = False

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        super().feed_robot_info(robot_id, jnt_to_id)
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

    def set_jpos(self, pos, wait=True, ignore_physics=False):
        """
        Set the gripper position.

        Args:
            pos (float): joint position.
            wait (bool): wait until the joint position is set
                to the target position.

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits.
        """
        joint_name = self.jnt_names[0]
        tgt_pos = arutil.clamp(
            pos,
            min(self.gripper_open_angle, self.gripper_close_angle),
            max(self.gripper_open_angle, self.gripper_close_angle))

        jnt_id = self.jnt_to_id[joint_name]
        if ignore_physics:
            self._zero_vel_mode()
            mic_pos = self._mimic_gripper(pos)
            self._hard_reset(mic_pos)
            success = True
        else:
            self._pb.setJointMotorControl2(self.robot_id,
                                           jnt_id,
                                           self._pb.POSITION_CONTROL,
                                           targetPosition=tgt_pos,
                                           force=self._max_torque)
            if not self._pb.in_realtime_mode():
                self._set_rest_joints(tgt_pos)

            success = False
            if self._pb.in_realtime_mode() and wait:
                success = wait_to_reach_jnt_goal(
                    tgt_pos,
                    get_func=self.get_jpos,
                    joint_name=joint_name,
                    get_func_derv=self.get_jvel,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_ERROR
                )
        return success

    def get_jpos(self):
        """
        Return the joint position(s) of the gripper.

        Returns:
            float: joint position.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        pos = self._pb.getJointState(self.robot_id, jnt_id)[0]
        return pos

    def get_jvel(self):
        """
        Return the joint velocity of the gripper.

        Returns:
            float: joint velocity.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        vel = self._pb.getJointState(self.robot_id, jnt_id)[1]
        return vel

    def _mimic_gripper(self, joint_val):
        """
        Given the value for the first joint,
        mimic the joint values for the rest joints.
        """
        jnt_vals = [joint_val]
        for i in range(1, len(self.jnt_names)):
            jnt_vals.append(joint_val * self._gripper_mimic_coeff[i])
        return jnt_vals

    def _th_mimic_gripper(self):
        """
        Make all the other joints of the gripper
        follow the motion of the first joint of the gripper.
        """
        while True:
            if self._is_activated and self._pb.in_realtime_mode():
                self._set_rest_joints()
            time.sleep(0.005)

    def _set_rest_joints(self, gripper_pos=None):
        max_torq = self._max_torque
        max_torques = [max_torq] * (len(self.jnt_names) - 1)
        if gripper_pos is None:
            gripper_pos = self.get_jpos()
        gripper_poss = self._mimic_gripper(gripper_pos)[1:]
        gripper_vels = [0.0] * len(max_torques)
        self._pb.setJointMotorControlArray(self.robot_id,
                                           self.gripper_jnt_ids[1:],
                                           self._pb.POSITION_CONTROL,
                                           targetPositions=gripper_poss,
                                           targetVelocities=gripper_vels,
                                           forces=max_torques)
