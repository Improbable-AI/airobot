import numpy as np

import airobot as ar
import matplotlib.pyplot as plt


def main():
    """
    This function demonstrates how to setup camera
    and get rgb/depth images.
    """
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True})
    focus_pt = [0, 0, 1]  # ([x, y, z])
    robot.setup_camera(focus_pt=focus_pt,
                       dist=3,
                       yaw=0,
                       pitch=0,
                       roll=0)
    robot.go_home()
    rgba, depth = robot.get_images(get_rgb=True,
                                   get_depth=True)
    plt.imshow(rgba)
    print('Maximum Depth (m): %f' % np.max(depth))
    print('Minimum Depth (m): %f' % np.min(depth))
    plt.show()


if __name__ == '__main__':
    main()
