import json
import os
import signal
import sys
import time

import numpy as np
import open3d
import rospkg

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
    rospack = rospkg.RosPack()
    data_path = rospack.get_path('hand_eye_calibration')
    calib_file_path = os.path.join(data_path, 'calib_base_to_cam.json')
    with open(calib_file_path, 'r') as f:
        calib_data = json.load(f)
    cam_pos = np.array(calib_data['b_c_transform']['position'])
    cam_ori = np.array(calib_data['b_c_transform']['orientation'])

    robot.camera.set_cam_ext(cam_pos, cam_ori)
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")
    pcd = open3d.geometry.PointCloud()
    pts, colors = robot.camera.get_pcd(in_world=True,
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
