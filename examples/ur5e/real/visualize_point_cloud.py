import signal
import sys
import time

import numpy as np
import open3d

import airobot as ar


def signal_handler(sig, frame):
    """
    Capture exit signal from the keyboard
    """
    print('Exit')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main():
    """
    Visualize the point cloud from the RGBD camera
    """
    robot = ar.create_robot('ur5e',
                            pb=False,
                            robot_cfg={'use_cam': True,
                                       'use_arm': False})
    cam_pos = np.array([1.216, -0.118, 0.610])
    cam_ori = np.array([-0.471, -0.0321, 0.880, 0.0294])
    robot.camera.set_cam_ext(cam_pos, cam_ori)
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")
    pcd = open3d.geometry.PointCloud()
    pts, colors = robot.camera.get_pcd(in_world=False,
                                       filter_depth=False)
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)
    while True:
        pts, colors = robot.camera.get_pcd(in_world=True,
                                           filter_depth=False)
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
