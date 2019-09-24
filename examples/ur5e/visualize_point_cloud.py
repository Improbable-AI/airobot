import open3d
import sys

import open3d

import airobot as ar
import signal

def signal_handler(sig, frame):
    print('Exit')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def main():
    robot = ar.create_robot('ur5e',
                            pb=False)
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")
    pcd = open3d.geometry.PointCloud()

    vis.add_geometry(pcd)
    while True:
        pcd.clear()
        rgb, depth = robot.camera.get_images()
        pts, colors = robot.camera.get_pcd(depth, rgb,
                                           in_world=False,
                                           filter_depth=False)
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()


if __name__ == '__main__':
    main()
