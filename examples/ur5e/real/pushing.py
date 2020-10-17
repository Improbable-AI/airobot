# modified from PyRobot LoCoBot pushing example

from __future__ import print_function

import argparse
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from sklearn.cluster import DBSCAN

from airobot import Robot
from airobot.utils.ros_util import read_cam_ext


def signal_handler(sig, frame):
    """
    Capture keyboard interruption signal.
    """
    print('Exit')
    sys.exit(0)


# range for x and y coordinates
Y_range = [-0.7, 0.7]
X_range = [0, 1.1]

signal.signal(signal.SIGINT, signal_handler)


def filter_points(pts, colors, z_lowest=0.01):
    """
    Filter the point cloud based on the x, y, z range.
    X and Y range are defined as the global variables.
    Z range only has a lower bound which is z_lowest.

    Args:
        pts (np.ndarray): point cloud (shape: [N, 3]).
        colors (np.ndarray): color of the point cloud (shape: [N, 3]).
        z_lowest (float): the smallest acceptable z coordinate.

    Returns:
        np.ndarray: point cloud after filter (shape: [N, 3]).
        np.ndarray: color of the points (shape: [N, 3]).
    """
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


def segment_objects(pts):
    """
    Use DBSCAN clustering algorithm to cluster the 2D points.

    Args:
        pts (np.ndarray): 2D points [x, y], (shape: [N, 2]).

    Returns:
        2-element tuple (if `get_seg` is False) containing

        - np.ndarray: Cluster labels for each point in the
          dataset given to fit(). Noisy samples are given
          the label -1 (shape: [N]).
        - np.ndarray: A binary array that's of the same shape
          as the labels and it indicates whether the sample
          point is a core sample (shape: [N]).

    """
    db = DBSCAN(eps=0.02, min_samples=15).fit(pts)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    return labels, core_samples_mask


def draw_segments(pts, labels, core_samples_mask):
    """
    Draw the 2D clustering result and save it as an image.

    Also, it filters the labels so that only the labels with
    the number of samples greater than a threshold will be remained.

    Args:
        pts (np.ndarray): 2D points [x, y], (shape: [N, 2])
        labels (np.ndarray): Cluster labels for each point in the
            dataset given to fit(). Noisy samples are given
             the label -1 (shape: [N])
        core_samples_mask (np.ndarray): A binary array that's of the same shape
            as the labels and it indicates whether the sample
             point is a core sample (shape: [N])

    Returns:
        list: unique and useful labels (labels with
        the number of samples greater than a threshold will be remained).
    """
    num_threshold = 800
    plt.clf()
    plt.scatter(pts[:, 0], pts[:, 1])
    plt.xlim(Y_range[0], Y_range[1])
    plt.ylim(X_range[0], X_range[1])
    plt.xlabel('Y axis of the base link')
    plt.ylabel('X axis of the base link')
    plt.savefig('raw_pts.png')
    plt.clf()
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    unique_labels = set(labels)
    useful_labels = []
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    print('Estimated number of clusters: %d' % n_clusters_)
    for k, col in zip(unique_labels, colors):
        if k == -1:
            continue
        class_member_mask = (labels == k)

        xy = pts[class_member_mask & core_samples_mask]
        print('Label:[%d]   # of pts:%d' % (k, xy.shape[0]))
        if xy.shape[0] > num_threshold:
            useful_labels.append(k)
        plt.plot(xy[:, 1], xy[:, 0], 'o', markerfacecolor=tuple(col),
                 markersize=1)
        plt.xlim(Y_range[0], Y_range[1])
        plt.ylim(X_range[0], X_range[1])
        plt.xlabel('Y axis of the base link')
        plt.ylabel('X axis of the base link')
    plt.savefig('seg_pts.png')
    print('Number of clusters after filtering:', len(useful_labels))
    return useful_labels


def sample_pt(pts, labels, useful_labelset, z_lowest):
    """
    Get the end effector's initial position where the gripper
    will be above the object and the start position on the
    table to push the object, and the center point
    of the obejct's point cloud.


    Args:
        pts (np.ndarray): 3D point cloud (shape: [N, 3]).
        labels (np.ndarray): Cluster labels for each point in the
            dataset. Noisy samples are given the label -1 (shape: [N]).
        useful_labelset (list): useful and unique label set.
        z_lowest (float): minimum acceptable z coordinate for the 3D points.

    Returns:
        3-element tuple (if `get_seg` is False) containing

        - np.ndarray: initial positin for the gripper so that the gripper
          will be directly above the object (shape: [3,]).
        - np.ndarray: start position that's close to the object
          for pushing (shape: [3,]).
        - np.ndarray: center point of the object's point cloud (shape: [3,]).

    """
    tgt_label = np.random.choice(useful_labelset, 1)[0]
    tgt_pts = pts[labels == tgt_label]
    center = np.mean(tgt_pts, axis=0)
    bbox_xy_min = np.amin(tgt_pts[:, :2], axis=0) - 0.03
    bbox_xy_max = np.amax(tgt_pts[:, :2], axis=0) + 0.03
    bbox_xy = np.stack((bbox_xy_min, bbox_xy_max), axis=0)
    tgt_edge = np.random.choice(2, 1)[0]
    if tgt_edge == 0:
        tgt_x = np.random.uniform(bbox_xy[0, 0], bbox_xy[1, 0], 1)[0]
        tgt_y = bbox_xy[int(np.random.choice(2, 1)[0]), 1]
    else:
        tgt_y = np.random.uniform(bbox_xy[0, 1], bbox_xy[1, 1], 1)[0]
        tgt_x = bbox_xy[int(np.random.choice(2, 1)[0]), 0]
    tgt_z = max(np.min(tgt_pts[:, 2]), z_lowest - 0.02)
    center[2] = tgt_z
    mid_pt = np.array([tgt_x, tgt_y, tgt_z])

    start_pt = np.array([tgt_x, tgt_y, 0.2])
    return start_pt, mid_pt, center


def push(bot, reset_pos, z_lowest=-0.17):
    """
    Start pushing objects.

    Args:
        bot (airobot.Robot): robot instance.
        reset_pos (np.ndarray): reset position of the gripper (shape: [3,]).
        z_lowest (float): minimum acceptable z coordinate
            for the 3D points of objects.
            This is a simple way to filter out the table.
            Basically, all points with
            z coordinate (e.g., the table) less than
            this value will be removed.

    """
    cur_pos = bot.arm.get_ee_pose()[0]
    bot.arm.move_ee_xyz(reset_pos - cur_pos)
    print('Getting point cloud')
    pts, colors = bot.cam.get_pcd(in_world=True,
                                  filter_depth=True)
    print('Selecting points of interest')
    pts, colors = filter_points(pts, colors, z_lowest=z_lowest)
    X = pts[:, :2]
    labels, core_samples_mask = segment_objects(X)
    useful_labelset = draw_segments(X, labels, core_samples_mask)
    start_pt, mid_pt, center = sample_pt(pts, labels, useful_labelset,
                                         z_lowest=z_lowest)

    print("Going to: ", start_pt.tolist())
    result = bot.arm.move_ee_xyz(start_pt - reset_pos)
    if not result:
        return
    down_disp = mid_pt - start_pt
    bot.arm.move_ee_xyz(down_disp)
    hor_disp = 2 * (center - mid_pt)
    bot.arm.move_ee_xyz(hor_disp)
    bot.arm.move_ee_xyz([0, 0, 0.2])


def main():
    """
    Push the objects on the table randomly.

    The robot will first go to a reset position so that the robot
    arm does not block the camera's view. And a clustering alogrithm
    is used to cluster the 3D point cloud of the objects on the table.
    Then the gripper pushs a randomly selected object on the table.
    And the gripper goes back to a reset position.
    """
    parser = argparse.ArgumentParser(description='Argument Parser')
    parser.add_argument('--z_min', type=float, default=-0.15,
                        help='minimium acceptable z value')
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)
    bot = Robot('ur5e_2f140', pb=False, use_cam=True)

    cam_pos, cam_ori = read_cam_ext('ur5e')
    bot.cam.set_cam_ext(cam_pos, cam_ori)
    bot.arm.go_home()
    bot.arm.eetool.activate()
    bot.arm.eetool.close()

    pre_jnts = [1.57, -1.66, -1.92, -1.12, 1.57, 0]
    bot.arm.set_jpos(pre_jnts)
    time.sleep(1)
    ee_pose = bot.arm.get_ee_pose()
    reset_pos = ee_pose[0]
    print('Reset pos:', reset_pos)

    while not rospy.is_shutdown():
        try:
            push(bot, reset_pos, z_lowest=args.z_min)
        except Exception as e:
            print(e)
        time.sleep(1)


if __name__ == "__main__":
    main()
