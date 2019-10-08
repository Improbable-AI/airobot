from yacs.config import CfgNode as CN

from airobot.cfgs.arm import get_cfg_defaults
from airobot.cfgs.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.robotiq2f140 import get_robotiq2f140_cfg

_C = get_cfg_defaults()
_C.ROBOT_DESCRIPTION = '/robot_description'
# _C.PYBULLET_URDF = 'yumi_nogrippers.urdf'
_C.PYBULLET_URDF = 'yumi_gelsight.urdf'
_C.MOVEGROUP_NAME = 'manipulator'
_C.ROSTOPIC_JOINT_STATES = '/joint_states'
# base frame for the arm
_C.ROBOT_BASE_FRAME = 'base'
# end-effector frame of the arm
_C.ROBOT_EE_FRAME_RIGHT = 'yumi_link_7_r'
_C.ROBOT_EE_FRAME_LEFT = 'yumi_link_7_l'
_C.ROBOT_EE_FRAME_JOINT_RIGHT = 'yumi_joint_6_r'
_C.ROBOT_EE_FRAME_JOINT_LEFT = 'yumi_joint_6_l'
_C.JOINT_SPEED_TOPIC = '/joint_speed'
_C.URSCRIPT_TOPIC = '/ur_driver/URScript'
# inverse kinematics position tolerance (m)
_C.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.IK_ORIENTATION_TOLERANCE = 0.1
_C.HOME_POSITION = [0.413, -1.325, -0.053, -0.484, 0.841, -1.546, -1.040,
                    -0.473, -1.450, 0.031, 0.513, 0.77, -1.669, 1.091]
# _C.HOME_POSITION = [3.14, -1.513, 2.020, -2.078, -1.571, 0]
_C.MAX_JOINT_ERROR = 0.01
_C.MAX_JOINT_VEL_ERROR = 0.1
_C.MAX_EE_POSITION_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.MAX_EE_ORIENTATION_ERROR = 0.02
_C.TIMEOUT_LIMIT = 10
_C.CAM_SIM = get_sim_cam_cfg()
_C.CAM_REALSENSE = get_realsense_cam_cfg()
_C.CAM_KINECT = CN()
# _C.GRIPPER = get_robotiq2f140_cfg()


def get_cfg():
    return _C.clone()
