from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.OPEN_ANGLE = 0.04
# joint angle when the gripper is fully closed
_C.CLOSE_ANGLE = 0.0

# for the gripper in pybullet
_C.JOINT_NAMES = [
    'panda_finger_joint1', 'panda_finger_joint2'
]
_C.MAX_TORQUE = 20.0


def get_frankagripper_cfg():
    return _C.clone()
