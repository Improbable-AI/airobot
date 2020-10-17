import signal
import sys
import time

import open3d

from airobot import Robot
from airobot.utils.ros_util import read_cam_ext


def signal_handler(sig, frame):
    """
    Capture exit signal from the keyboard.
    """
    print('Exit')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main():
    """
    Visualize the point cloud from the RGBD camera.
    """
    robot = Robot('ur5e_2f140',
                  pb=False, use_cam=True, use_arm=False)

    cam_pos, cam_ori = read_cam_ext('ur5e')
    robot.cam.set_cam_ext(cam_pos, cam_ori)
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")
    pcd = open3d.geometry.PointCloud()
    pts, colors = robot.cam.get_pcd(in_world=True,
                                    filter_depth=False)
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)
    while True:
        pts, colors = robot.cam.get_pcd(in_world=True,
                                        filter_depth=False)
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
