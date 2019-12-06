import numpy as np
from gym import spaces

from airobot import Robot
from airobot.utils.common import euler2quat
from airobot.utils.pb_util import load_geom
from airobot.utils.pb_util import load_urdf
from airobot.utils.pb_util import step_simulation


class URRobotGym:
    def __init__(self, action_repeat=10, render=True):
        self._action_repeat = action_repeat
        self.robot = Robot('ur5e', arm_cfg={'render': render,
                                            'rt_simulation': False})
        self.ee_ori = [-np.sqrt(2) / 2, np.sqrt(2) / 2, 0, 0]
        self._action_bound = 1.0
        self._ee_pos_scale = 0.02
        self._ee_ori_scale = np.pi / 36.0
        self._action_high = np.array([self._action_bound] * 5)
        self.action_space = spaces.Box(low=-self._action_high,
                                       high=self._action_high,
                                       dtype=np.float32)
        self.reset()

    def reset(self):
        self.robot.arm.reset()
        self.robot.arm.go_home(ignore_physics=True)
        ori = euler2quat([0, 0, np.pi / 2])
        self.table_id = load_urdf('table/table.urdf',
                                  [.5, 0, 0.4],
                                  ori,
                                  scaling=0.9)
        self.box_id = load_geom('box', size=0.05, mass=1,
                                base_pos=[0.5, 0.12, 1.0],
                                rgba=[1, 0, 0, 1])
        self.gripper_ori = self.robot.arm.get_jpos('wrist_3_joint')

    def apply_action(self, action):
        if not isinstance(action, np.ndarray):
            action = np.array(action).flatten()
        if action.size != 5:
            raise ValueError('Action should be [d_x, d_y, d_z, '
                             'd_angle, open/close gripper].')
        pos, quat, rot_mat, euler = self.robot.arm.get_ee_pose()
        pos += action[:3] * self._ee_pos_scale
        self.gripper_ori += action[3] * self._ee_ori_scale
        self.gripper_ori = (self.gripper_ori + np.pi) % (2 * np.pi) - np.pi
        jnt_pos = self.robot.arm.compute_ik(pos, ori=self.ee_ori)
        jnt_pos[-1] = self.gripper_ori
        for step in range(self._action_repeat):
            self.robot.arm.set_jpos(jnt_pos)
            step_simulation()

    def render(self, **kwargs):
        robot_base = self.robot.arm.robot_base_pos
        self.robot.cam.setup_camera(focus_pt=robot_base,
                                    dist=2,
                                    yaw=0,
                                    pitch=60,
                                    roll=0)

        rgb, _ = self.robot.cam.get_images(get_rgb=True,
                                           get_depth=False)
        return rgb


def main():
    env = URRobotGym(render=True)
    from IPython import embed
    embed()


if __name__ == '__main__':
    main()
