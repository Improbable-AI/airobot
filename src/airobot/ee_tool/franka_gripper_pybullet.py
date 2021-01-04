import airobot.utils.common as arutil
from airobot.ee_tool.simple_gripper_pybullet import SimpleGripperPybullet
from airobot.utils.arm_util import wait_to_reach_jnt_goal


class FrankaGripperPybullet(SimpleGripperPybullet):
    """
    Class for interfacing with a Franka gripper in pybullet.

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
        super(FrankaGripperPybullet, self).__init__(cfgs=cfgs,
                                                    pb_client=pb_client)

    def _setup_gripper(self):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        c = self._pb.createConstraint(self.robot_id,
                                      self.gripper_jnt_ids[0],
                                      self.robot_id,
                                      self.gripper_jnt_ids[1],
                                      jointType=self._pb.JOINT_GEAR,
                                      jointAxis=[1, 0, 0],
                                      parentFramePosition=[0, 0, 0],
                                      childFramePosition=[0, 0, 0])
        self._pb.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        super().feed_robot_info(robot_id, jnt_to_id)
        self._setup_gripper()

    def set_pos(self, pos, wait=True, ignore_physics=False):
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
        tgt_pos = arutil.clamp(
            pos,
            min(self.gripper_open_angle, self.gripper_close_angle),
            max(self.gripper_open_angle, self.gripper_close_angle))
        tgt_pos = [tgt_pos] * len(self.gripper_jnt_ids)
        if ignore_physics:
            self._zero_vel_mode()
            self._hard_reset(pos)
            success = True
        else:
            self._pb.setJointMotorControlArray(self.robot_id,
                                               self.gripper_jnt_ids,
                                               self._pb.POSITION_CONTROL,
                                               targetPositions=tgt_pos,
                                               forces=[self._max_torque] * len(self.gripper_jnt_ids))

            success = False
            if self._pb.in_realtime_mode() and wait:
                success = wait_to_reach_jnt_goal(
                    tgt_pos[0],
                    get_func=self.get_jpos,
                    joint_name=self.jnt_names[0],
                    get_func_derv=self.get_jvel,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_ERROR
                )
        return success
