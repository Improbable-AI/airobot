from airobot.cfgs.arm import get_cfg_defaults

_C = get_cfg_defaults()

_C.URDF = 'ur5e_2f140_pybullet.urdf'
_C.MAX_JOINT_ERROR = 0.01
_C.TIMEOUT_LIMIT = 5


def get_cfg():
    return _C.clone()
