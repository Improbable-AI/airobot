from airobot.ee_tool.simple_gripper_mimic_pybullet import SimpleGripperMimicPybullet


class Robotiq2F140Pybullet(SimpleGripperMimicPybullet):
    """
    Class for interfacing with a Robotiq 2F140 gripper when
    it is attached to UR5e arm in pybullet.

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
        super(Robotiq2F140Pybullet, self).__init__(cfgs=cfgs,
                                                   pb_client=pb_client)

    def _setup_gripper(self):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        self._pb.changeDynamics(self.robot_id,
                                self.jnt_to_id['left_inner_finger_pad_joint'],
                                lateralFriction=2.0,
                                spinningFriction=1.0,
                                rollingFriction=1.0)
        self._pb.changeDynamics(self.robot_id,
                                self.jnt_to_id['right_inner_finger_pad_joint'],
                                lateralFriction=2.0,
                                spinningFriction=1.0,
                                rollingFriction=1.0)

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        super().feed_robot_info(robot_id, jnt_to_id)
        self._setup_gripper()
