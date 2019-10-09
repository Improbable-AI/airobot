from __future__ import print_function

import cv2
import rospy
import airobot as ar

if __name__ == "__main__":
    robot = ar.create_robot('ur5e',
                            pb=False,
                            robot_cfg={'use_cam': True,
                                       'use_arm': False})
    while not rospy.is_shutdown():
        rgb, depth = robot.camera.get_images(get_rgb=True, get_depth=True)
        cv2.imshow('Color', rgb[:, :, ::-1])
        # cv2.imshow('Depth', depth)
        cv2.waitKey(10)
