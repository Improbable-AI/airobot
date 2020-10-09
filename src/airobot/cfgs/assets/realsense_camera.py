from yacs.config import CfgNode as CN

_C = CN()
# # topic name of the camera info
# _C.ROSTOPIC_CAMERA_INFO = 'camera/color/camera_info'
# # topic name of the RGB images
# _C.ROSTOPIC_CAMERA_RGB = 'camera/color/image_rect_color'
# # topic name of the depth images
# _C.ROSTOPIC_CAMERA_DEPTH = 'camera/aligned_depth_to_color/image_raw'

# topic name of the camera info
_C.ROSTOPIC_CAMERA_INFO = 'cam_2/color/camera_info'
# topic name of the RGB images
_C.ROSTOPIC_CAMERA_RGB = 'cam_2/color/image_rect_color'
# topic name of the depth images
_C.ROSTOPIC_CAMERA_DEPTH = 'cam_2/aligned_depth_to_color/image_raw'

# minimum depth values to be considered as valid (m)
_C.DEPTH_MIN = 0.2
# maximum depth values to be considered as valid (m)
_C.DEPTH_MAX = 2
# scale factor to map depth image values to real depth values (m)
_C.DEPTH_SCALE = 0.001


def get_realsense_cam_cfg():
    return _C.clone()
