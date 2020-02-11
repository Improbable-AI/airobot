"""
An example of UR robot with dual cameras in pybullet.
"""

from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np
from airobot import Robot
from airobot import log_info
from airobot.sensor.camera.rgbdcam_pybullet import RGBDCameraPybullet
from airobot.utils.common import ang_in_mpi_ppi
from airobot.utils.common import clamp
from airobot.utils.common import euler2quat
from airobot.utils.common import quat_multiply
from airobot.utils.common import rotvec2quat
from gym import spaces
from yacs.config import CfgNode as CN


class URRobotGym:
    def __init__(self, action_repeat=10, gui=True):
        self._action_repeat = action_repeat
        self.robot = Robot('ur5e_2f140',
                           pb_cfg={'gui': gui,
                                   'realtime': False})

        self._ee_pos_scale = 0.02
        self._ee_ori_scale = np.pi / 36.0

        self.cams = []
        pb_client = self.robot.pb_client
        for i in range(2):
            self.cams.append(RGBDCameraPybullet(cfgs=self._camera_cfgs(),
                                                pb_client=pb_client))
        self._setup_cameras()
        self.reset()

        self._action_bound = 1.0
        self._action_high = np.array([self._action_bound] * 5)
        self.action_space = spaces.Box(low=-self._action_high,
                                       high=self._action_high,
                                       dtype=np.float32)
        ob = self._get_obs()
        self.observation_space = spaces.Box(low=0,
                                            high=255,
                                            shape=ob.shape,
                                            dtype=np.uint8)

    def reset(self):
        self.robot.arm.reset()
        self.robot.arm.go_home(ignore_physics=True)
        ori = euler2quat([0, 0, np.pi / 2])
        self.table_id = self.robot.pb_client.load_urdf('table/table.urdf',
                                                       [.5, 0, 0.4],
                                                       ori,
                                                       scaling=0.9)
        self.box_id = self.robot.pb_client.load_geom('box', size=0.05, mass=1,
                                                     base_pos=[0.5, 0.12, 1.0],
                                                     rgba=[1, 0, 0, 1])
        self.ref_ee_ori = self.robot.arm.get_ee_pose()[1]
        self.gripper_ori = 0
        return self._get_obs()

    def step(self, action):
        self.apply_action(action)
        ob = self._get_obs()
        done = False
        info = dict()
        reward = -1
        return ob, reward, done, info

    def _get_obs(self):
        """

        Returns:
            np.ndarray: observations from two cameras.
            The two images are concatenated together.
            The returned observation shape is [2H, W, 3].
        """
        rgbs = []
        for cam in self.cams:
            rgb, _ = cam.get_images(get_rgb=True, get_depth=False)
            rgbs.append(deepcopy(rgb))
        return np.concatenate(rgbs, axis=0)

    def get_robot_state(self):
        jpos = self.robot.arm.get_jpos()
        jvel = self.robot.arm.get_jvel()
        state = jpos + jvel
        return state

    def apply_action(self, action):
        if not isinstance(action, np.ndarray):
            action = np.array(action).flatten()
        if action.size != 5:
            raise ValueError('Action should be [d_x, d_y, d_z, '
                             'd_angle, open/close gripper].')
        pos, quat, rot_mat, euler = self.robot.arm.get_ee_pose()
        pos += action[:3] * self._ee_pos_scale

        self.gripper_ori += action[3] * self._ee_ori_scale
        self.gripper_ori = ang_in_mpi_ppi(self.gripper_ori)
        rot_vec = np.array([0, 0, 1]) * self.gripper_ori
        rot_quat = rotvec2quat(rot_vec)
        ee_ori = quat_multiply(self.ref_ee_ori, rot_quat)
        jnt_pos = self.robot.arm.compute_ik(pos, ori=ee_ori)
        gripper_ang = self._scale_gripper_angle(action[4])

        for step in range(self._action_repeat):
            self.robot.arm.set_jpos(jnt_pos)
            self.robot.arm.eetool.set_pos(gripper_ang)
            self.robot.pb_client.stepSimulation()

    def _camera_cfgs(self):
        _C = CN()
        _C.ZNEAR = 0.01
        _C.ZFAR = 10
        _C.WIDTH = 640
        _C.HEIGHT = 480
        _C.FOV = 60
        _ROOT_C = CN()
        _ROOT_C.CAM = CN()
        _ROOT_C.CAM.SIM = _C
        return _ROOT_C.clone()

    def _setup_cameras(self):
        self.cams[0].setup_camera(focus_pt=[0.5, 0., 1.0],
                                  dist=2,
                                  yaw=-90,
                                  pitch=-45,
                                  roll=0)
        self.cams[1].setup_camera(focus_pt=[0.5, 0., 1.0],
                                  dist=2,
                                  yaw=90,
                                  pitch=-45,
                                  roll=0)

    def _scale_gripper_angle(self, command):
        """
        Convert the command in [-1, 1] to the actual gripper angle.
        command = -1 means open the gripper.
        command = 1 means close the gripper.

        Args:
            command (float): a value between -1 and 1.
                -1 means open the gripper.
                1 means close the gripper.

        Returns:
            float: the actual gripper angle
            corresponding to the command.
        """
        command = clamp(command, -1.0, 1.0)
        close_ang = self.robot.arm.eetool.gripper_close_angle
        open_ang = self.robot.arm.eetool.gripper_open_angle
        cmd_ang = (command + 1) / 2.0 * (close_ang - open_ang) + open_ang
        return cmd_ang

    def render(self, **kwargs):
        robot_base = self.robot.arm.robot_base_pos
        self.robot.cam.setup_camera(focus_pt=robot_base,
                                    dist=3,
                                    yaw=55,
                                    pitch=-30,
                                    roll=0)

        rgb, _ = self.robot.cam.get_images(get_rgb=True,
                                           get_depth=False)
        return rgb


def main():
    env = URRobotGym(gui=False)
    ob = env.reset()
    log_info('\nThe action is a 5-dim vector A. \n'
             'The range of each element is [-1, 1].\n'
             'A[0] is dx, A[1] is dy, A[2] is dz;\n'
             'A[3] is the delta of the gripper orientation;\n'
             'A[4] is the gripper opening angle, \n'
             '-1 means opening the gripper, \n'
             '1 means closing the gripper.\n')
    image = plt.imshow(ob, interpolation='none',
                       animated=True, label="cam")
    ax = plt.gca()
    for j in range(2):
        for i in range(10):
            ob, reward, done, info = env.step([1, 0, 0, 0, -1])
            image.set_data(ob)
            ax.plot([0])
            plt.pause(0.05)
        for i in range(10):
            ob, reward, done, info = env.step([-1, 0, 0, 0, -1])
            image.set_data(ob)
            ax.plot([0])
            plt.pause(0.05)


if __name__ == '__main__':
    main()
