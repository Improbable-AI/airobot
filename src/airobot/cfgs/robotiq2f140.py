from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.OPEN_ANGLE = 0
# joint angle when the gripper is fully closed
_C.CLOSE_ANGLE = 0.7

# default host and port for socket used to
# communicate with the gripper through the
# UR controller
_C.SOCKET_HOST = "127.0.0.1"
_C.SOCKET_PORT = 63352

def get_robotiq2f140_cfg():
    return _C.clone()
