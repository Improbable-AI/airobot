import signal
import sys
import time

import open3d

from airobot import Robot


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
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()
    robot.cam.setup_camera(focus_pt=robot.arm.robot_base_pos,
                           dist=3,
                           yaw=55,
                           pitch=-30,
                           roll=0)
    depth_max = 5.0
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")

    pcd = open3d.geometry.PointCloud()
    pts, colors = robot.cam.get_pcd(in_world=True,
                                    filter_depth=True,
                                    depth_max=depth_max)
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)
    while True:
        pts, colors = robot.cam.get_pcd(in_world=True,
                                        filter_depth=True,
                                        depth_max=depth_max)
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
