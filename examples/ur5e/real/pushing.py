# modified from PyRobot LoCoBot pushing example

from __future__ import print_function

import argparse
import signal
import sys
import time
import os
import json
import rospkg

import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

import airobot as ar
from IPython import embed


def signal_handler(sig, frame):
    print('Exit')
    sys.exit(0)

Y_range = [-0.7, 0.7]
X_range = [0, 1.1]


signal.signal(signal.SIGINT, signal_handler)


def filter_points(pts, colors, z_lowest=0.01):
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
    db = DBSCAN(eps=0.02, min_samples=15).fit(pts)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    return labels, core_samples_mask


def draw_segments(pts, labels, core_samples_mask):
    num_threshold = 400
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
            # Black used for noise.
            col = [0, 0, 0, 1]
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
    tgt_z = max(np.min(tgt_pts[:, 2])+0.01, z_lowest-0.02)
    center[2] = tgt_z
    mid_pt = np.array([tgt_x, tgt_y, tgt_z])

    start_pt = np.array([tgt_x, tgt_y, 0.2])
    return start_pt, mid_pt, center


def push(bot, reset_pos, z_lowest=-0.17):
    cur_pos = bot.get_ee_pose()[0]
    # if ee_pose[0][2] < 0.20:
    #     bot.go_home()
    bot.move_ee_xyz(reset_pos - cur_pos)
    pts, colors = bot.camera.get_pcd(in_world=True,
                                     filter_depth=True)
    pts, colors = filter_points(pts, colors, z_lowest=z_lowest)
    X = pts[:, :2]
    labels, core_samples_mask = segment_objects(X)
    useful_labelset = draw_segments(X, labels, core_samples_mask)
    start_pt, mid_pt, center = sample_pt(pts, labels, useful_labelset,
                                         z_lowest=z_lowest)
    # embed()
    print("Going to: ", start_pt.tolist())
    result = bot.move_ee_xyz(start_pt - reset_pos)
    # result = bot.set_ee_pose(pos=start_pt, ori=[-0.7071, 0.7071, 0, 0])
    if not result:
        return
    down_disp = mid_pt - start_pt
    bot.move_ee_xyz(down_disp)
    hor_disp = 2 * (center - mid_pt)
    bot.move_ee_xyz(hor_disp)
    bot.move_ee_xyz([0, 0, 0.2])


def main():
    parser = argparse.ArgumentParser(description='Argument Parser')
    parser.add_argument('--z_min', type=float, default=-0.15,
                        help='minimium acceptable z value')
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)
    bot = ar.create_robot('ur5e', pb=False, robot_cfg={'use_cam': True})

    rospack = rospkg.RosPack()
    data_path = rospack.get_path('hand_eye_calibration')
    calib_file_path = os.path.join(data_path, 'calib_base_to_cam.json')
    with open(calib_file_path, 'r') as f:
        calib_data = json.load(f)
    cam_pos = np.array(calib_data['b_c_transform']['position'])
    cam_ori = np.array(calib_data['b_c_transform']['orientation'])

    bot.camera.set_cam_ext(cam_pos, cam_ori)
    # bot.set_comm_mode(use_urscript=True)
    # bot.go_home()
    bot.gripper.activate()
    bot.gripper.close()

    pre_jnts = [1.57, -1.66, -1.92, -1.12, 1.57, 0]
    bot.set_jpos(pre_jnts)
    time.sleep(1)
    ee_pose = bot.get_ee_pose()
    reset_pos = ee_pose[0]

    while True:
        try:
            push(bot, reset_pos, z_lowest=args.z_min)
        except:
            pass
        time.sleep(1)


if __name__ == "__main__":
    main()
