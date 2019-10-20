import matplotlib.pyplot as plt
import numpy as np

import airobot as ar


def main():
    """
    This function demonstrates how the pitch angle (
     the pitch angle that is defined in robot.setup_camera) changes
    the camera view.
    """
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True})
    focus_pt = [0, 0, 1]  # ([x, y, z])
    robot.go_home()
    img = np.random.rand(480, 640)
    image = plt.imshow(img, interpolation='none',
                       animated=True, label="blah")
    ax = plt.gca()
    while True:
        for pitch in range(0, 360, 10):
            robot.setup_camera(focus_pt=focus_pt,
                               dist=3,
                               yaw=0,
                               pitch=pitch,
                               roll=0)

            rgba, depth = robot.get_images(get_rgb=True,
                                           get_depth=True)
            image.set_data(rgba)
            ax.plot([0])
            plt.pause(0.01)


if __name__ == '__main__':
    main()
