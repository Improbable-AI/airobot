from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.assets.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.assets.ur5e_arm import get_ur5e_arm_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = False

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'ur5e_stick_pybullet.urdf'

_C.ARM = get_ur5e_arm_cfg()

# update end-effector frame of the arm
_C.ARM.ROBOT_EE_FRAME = 'stick_tip'
_C.ARM.ROBOT_EE_FRAME_JOINT = 'stick_tip_joint'

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'


def get_cfg():
    return _C.clone()
