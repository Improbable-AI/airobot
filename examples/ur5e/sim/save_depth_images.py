import cv2
import numpy as np

from airobot import Robot
from airobot import log_info


def main():
    """
    This function demonstrates how to properly save depth
    images in python.
    """
    robot = Robot('ur5e', pb_cfg={'gui': False,
                                  'opengl_render': False})
    focus_pt = [0, 0, 1]
    robot.cam.setup_camera(focus_pt=focus_pt,
                           dist=3,
                           yaw=90,
                           pitch=0,
                           roll=0)
    robot.arm.go_home()
    rgb, depth, seg = robot.cam.get_images(get_rgb=True,
                                           get_depth=True,
                                           get_seg=True)
    log_info(f'Depth image min:{depth.min()} m, max: {depth.max()} m.')
    # we first convert the depth image into CV_16U format.
    # The maximum value for CV_16U is 65536.
    # Since png files only keep the integer values, we need to scale the
    # depth images to keep enough resolution.
    scale = 1000.0
    sdepth = depth * scale
    img_name = 'depth.png'
    # by converting the image data to CV_16U, the resolution of the
    # depth image data becomes 1 / scale m (0.001m in this case).
    cv2.imwrite(img_name, sdepth.astype(np.uint16))
    re_depth = cv2.imread(img_name, cv2.IMREAD_UNCHANGED)
    sre_depth = re_depth / scale
    log_info(f'Saved depth image min:{sre_depth.min()} m, '
             f'max: {sre_depth.max()} m.')


if __name__ == '__main__':
    main()
