# modified from PyRobot LoCoBot pushing example

from __future__ import print_function

import argparse
import signal
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

import airobot as ar


def signal_handler(sig, frame):
    print('Exit')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def filter_points(pts, colors, z_lowest=0.01):
    valid = pts[:, 2] > z_lowest
    valid = np.logical_and(valid,
                           pts[:, 0] < 0.5)
    valid = np.logical_and(valid,
                           pts[:, 1] < 0.4)
    valid = np.logical_and(valid,
                           pts[:, 1] > -0.4)
    pts = pts[valid]
    colors = colors[valid]
    return pts, colors


def segment_objects(pts):
    db = DBSCAN(eps=0.02, min_samples=15).fit(pts)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    return labels, core_samples_mask


def draw_segments(pts, labels, core_samples_mask):
    num_threshold = 400
    plt.clf()
    plt.scatter(pts[:, 0], pts[:, 1])
    plt.xlim(-0.4, 0.4)
    plt.ylim(0, 0.5)
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
            # Black used for noise.
            col = [0, 0, 0, 1]
            continue
        class_member_mask = (labels == k)

        xy = pts[class_member_mask & core_samples_mask]
        print('Label:[%d]   # of pts:%d' % (k, xy.shape[0]))
        if xy.shape[0] > num_threshold:
            useful_labels.append(k)
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markersize=1)
        plt.xlim(-0.4, 0.4)
        plt.ylim(0, 0.5)
        plt.xlabel('Y axis of the base link')
        plt.ylabel('X axis of the base link')
    plt.savefig('seg_pts.png')
    print('Number of clusters after filtering:', len(useful_labels))
    return useful_labels


def sample_pt(pts, labels, useful_labelset, z_lowest):
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
    tgt_z = max(center[2], z_lowest)
    center[2] = tgt_z
    mid_pt = np.array([tgt_x, tgt_y, tgt_z])

    start_pt = np.array([tgt_x, tgt_y, 0.2])
    return start_pt, mid_pt, center


def push(bot, z_lowest=-0.15):
    ee_pose = bot.get_ee_pose()
    # if ee_pose[0][2] < 0.20:
    #     bot.go_home()
    pre_jnts = [1.57, -1.66, -1.92, -1.12, 1.57, 0]
    self.robot.set_jpos(pre_jnts)
    pts, colors = bot.camera.get_pcd(in_cam=False)
    pts, colors = filter_points(pts, colors, z_lowest=z_lowest)
    X = pts[:, :2]
    labels, core_samples_mask = segment_objects(X)
    useful_labelset = draw_segments(X, labels, core_samples_mask)
    start_pt, mid_pt, center = sample_pt(pts, labels, useful_labelset,
                                         z_lowest=z_lowest)
    print("Going to: ", start_pt.tolist())
    result = bot.set_ee_pose(pos=start_pt, ori=[-0.7071, 0.7071, 0, 0])
    if not result:
        return
    down_disp = mid_pt - start_pt
    bot.move_ee_xyz(down_disp)
    hor_disp = 2 * (center - mid_pt)
    bot.move_ee_xyz(hor_disp)


def main():
    parser = argparse.ArgumentParser(description='Argument Parser')
    parser.add_argument('--floor_height', type=float, default=-0.15,
                        help='the z coordinate of the floor')
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)
    bot = ar.create_robot('ur5e', pb=False)
    cam_pos = np.array([1.216, -0.118, 0.610])
    cam_ori = np.array([-0.471, -0.0321, 0.880, 0.0294])
    bot.camera.set_cam_ext(cam_pos, cam_ori)
    bot.go_home()
    bot.gripper.activate()
    bot.gripper.close()

    while True:
        try:
            push(bot, z_lowest=args.floor_height)
        except:
            pass
        time.sleep(1)


if __name__ == "__main__":
    main()
