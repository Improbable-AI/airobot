import matplotlib.pyplot as plt
import numpy as np
from airobot import Robot
from airobot import log_info


def main():
    """
    This function demonstrates how to setup camera
    and get rgb/depth images and segmentation mask.
    """
    robot = Robot('ur5e', pb_cfg={'gui': True,
                                  'opengl_render': False})
    focus_pt = [0, 0, 1]  # ([x, y, z])
    robot.cam.setup_camera(focus_pt=focus_pt,
                           dist=3,
                           yaw=90,
                           pitch=0,
                           roll=0)
    robot.arm.go_home()
    rgb, depth, seg = robot.cam.get_images(get_rgb=True,
                                           get_depth=True,
                                           get_seg=True)
    plt.figure()
    plt.imshow(rgb)
    plt.figure()
    plt.imshow(depth * 25, cmap='gray', vmin=0, vmax=255)
    log_info('Maximum Depth (m): %f' % np.max(depth))
    log_info('Minimum Depth (m): %f' % np.min(depth))
    plt.figure()
    plt.imshow(seg)
    plt.show()


if __name__ == '__main__':
    main()
