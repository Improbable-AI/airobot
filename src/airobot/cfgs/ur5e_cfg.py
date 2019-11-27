from airobot.cfgs.default_configs import get_cfg_defaults
from airobot.cfgs.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.robotiq2f140 import get_robotiq2f140_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = True

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'ur5e_2f140_pybullet.urdf'

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.ARM.CLASS = 'UR5e'
_C.ARM.MOVEGROUP_NAME = 'manipulator'
_C.ARM.ROSTOPIC_JOINT_STATES = '/joint_states'

# https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
_C.ARM.MAX_TORQUES = [150, 150, 150, 28, 28, 28]
_C.ARM.JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]
# base frame for the arm
_C.ARM.ROBOT_BASE_FRAME = 'base'
# end-effector frame of the arm
_C.ARM.ROBOT_EE_FRAME = 'gripper_tip'
_C.ARM.ROBOT_EE_FRAME_JOINT = 'gripper_tip_joint'
_C.ARM.JOINT_SPEED_TOPIC = '/joint_speed'
_C.ARM.URSCRIPT_TOPIC = '/ur_driver/URScript'
# inverse kinematics position tolerance (m)
_C.ARM.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.ARM.IK_ORIENTATION_TOLERANCE = 0.05
_C.ARM.HOME_POSITION = [0, -1.66, -1.92, -1.12, 1.57, 0]
_C.ARM.MAX_JOINT_ERROR = 0.002
_C.ARM.MAX_JOINT_VEL_ERROR = 0.05
_C.ARM.MAX_EE_POS_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.ARM.MAX_EE_ORI_ERROR = 0.02
_C.ARM.TIMEOUT_LIMIT = 10

# reset position for the robot in pybullet
_C.ARM.PYBULLET_RESET_POS = [0, 0, 1]
# reset orientation (euler angles) for the robot in pybullet
_C.ARM.PYBULLET_RESET_ORI = [0, 0, 0]

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.EETOOL = get_robotiq2f140_cfg()
_C.EETOOL.CLASS = 'Robotiq2F140'


def get_cfg():
    return _C.clone()
