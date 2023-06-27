from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.franka_arm import get_franka_arm_cfg
from airobot.cfgs.assets.robotiq2f140 import get_robotiq2f140_cfg
from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = True

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'panda_2f140.urdf'

_C.ARM = get_franka_arm_cfg()

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.EETOOL = get_robotiq2f140_cfg()
_C.EETOOL.CLASS = 'Robotiq2F140'


def get_cfg():
    return _C.clone()
