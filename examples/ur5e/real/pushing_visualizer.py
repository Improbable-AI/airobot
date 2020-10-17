# modified from PyRobot LoCoBot pushing example

import argparse
import os
import signal
import sys
import time

import numpy as np
import open3d
from pushing import filter_points

from airobot import Robot
from airobot.utils.ros_util import read_cam_ext

sys.path.append(os.path.dirname(__file__))


def signal_handler(sig, frame):
    print('Exit')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main():
    """
    3D Point cloud visualization of the current scene.

    This is useful to see how does the point cloud that
    the pushing script works on look like
    """
    parser = argparse.ArgumentParser(description='Argument Parser')
    parser.add_argument('--z_min', type=float, default=-0.15,
                        help='minimium acceptable z value')
    args = parser.parse_args()
    np.set_printoptions(precision=4, suppress=True)

    robot = Robot('ur5e_2f140', pb=False, use_cam=True)
    pre_jnts = [1.57, -1.66, -1.92, -1.12, 1.57, 0]
    robot.arm.set_jpos(pre_jnts)

    cam_pos, cam_ori = read_cam_ext('ur5e')
    robot.cam.set_cam_ext(cam_pos, cam_ori)

    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")
    pcd = open3d.geometry.PointCloud()
    pts, colors = robot.cam.get_pcd(in_world=True,
                                    filter_depth=True)
    pts, colors = filter_points(pts, colors, z_lowest=args.z_min)
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    coord = open3d.geometry.TriangleMesh.create_coordinate_frame(1, [0, 0, 0])
    vis.add_geometry(coord)
    vis.add_geometry(pcd)
    while True:
        pts, colors = robot.cam.get_pcd(in_world=True,
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
