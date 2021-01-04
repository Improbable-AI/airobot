from airobot.ee_tool.simple_gripper_mimic_pybullet import SimpleGripperMimicPybullet


class YumiParallelJawPybullet(SimpleGripperMimicPybullet):
    """
    Class for interfacing with the standard Yumi
    parallel jaw gripper.

    Args:
        cfgs (YACS CfgNode): configurations for the gripper
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
        super(YumiParallelJawPybullet, self).__init__(cfgs=cfgs,
                                                      pb_client=pb_client)
