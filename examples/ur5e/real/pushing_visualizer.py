# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
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
    print('Exit')
    sys.exit(0)


Y_range = [-0.7, 0.7]
X_range = [0, 1.1]

signal.signal(signal.SIGINT, signal_handler)


def filter_points(pts, colors, z_lowest=-0.17):
    valid = pts[:, 2] > z_lowest
    valid = np.logical_and(valid,
                           pts[:, 0] > X_range[0])
    valid = np.logical_and(valid,
                           pts[:, 0] < X_range[1])
    valid = np.logical_and(valid,
                           pts[:, 1] < Y_range[1])
    valid = np.logical_and(valid,
                           pts[:, 1] > Y_range[0])
    pts = pts[valid]
    colors = colors[valid]
    return pts, colors


def main():
    parser = argparse.ArgumentParser(description='Argument Parser')
    parser.add_argument('--z_min', type=float, default=-0.15,
                        help='minimium acceptable z value')
    args = parser.parse_args()
    np.set_printoptions(precision=4, suppress=True)

    robot = ar.create_robot('ur5e', pb=False, robot_cfg={'use_cam': True})
    pre_jnts = [1.57, -1.66, -1.92, -1.12, 1.57, 0]
    robot.set_jpos(pre_jnts)

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
                                       filter_depth=True)
    pts, colors = filter_points(pts, colors, z_lowest=args.z_min)
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    coord = open3d.geometry.TriangleMesh.create_coordinate_frame(1, [0, 0, 0])
    vis.add_geometry(coord)
    vis.add_geometry(pcd)
    while True:
        pts, colors = robot.camera.get_pcd(in_world=True,
                                           filter_depth=True)
        pts, colors = filter_points(pts, colors, z_lowest=args.z_min)
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
