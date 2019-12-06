import time

import numpy as np

import airobot as ar
from airobot import Robot
from airobot.utils.common import euler2quat
from airobot.utils.pb_util import get_body_state
from airobot.utils.pb_util import load_geom
from airobot.utils.pb_util import load_urdf


def main():
    """
    This function shows an example of block stacking
    """
    np.set_printoptions(precision=4, suppress=True)
    robot = Robot('ur5e', arm_cfg={'render': True})
    success = robot.arm.go_home()
    if not success:
        ar.log_warn('Robot go_home failed!!!')
    ori = euler2quat([0, 0, np.pi / 2])
    load_urdf('table/table.urdf', [.5, 0, 0.4], ori, scaling=0.9)
    box_size = 0.05
    box_id1 = load_geom('box', size=box_size, mass=1,
                        base_pos=[.5, 0.12, 1.0], rgba=[1, 0, 0, 1])
    box_id2 = load_geom('box', size=box_size, mass=1,
                        base_pos=[0.3, 0.12, 1.0], rgba=[0, 0, 1, 1])
    robot.arm.eetool.open()
    obj_pos = get_body_state(box_id1)[0]
    move_dir = obj_pos - robot.arm.get_ee_pose()[0]
    move_dir[2] = 0
    robot.arm.move_ee_xyz(move_dir)
    move_dir = np.zeros(3)
    move_dir[2] = obj_pos[2] - robot.arm.get_ee_pose()[0][2]
    robot.arm.move_ee_xyz(move_dir)
    robot.arm.eetool.close(wait=False)
    robot.arm.move_ee_xyz([0, 0, 0.3])

    obj_pos = get_body_state(box_id2)[0]
    move_dir = obj_pos - robot.arm.get_ee_pose()[0]
    move_dir[2] = 0
    robot.arm.move_ee_xyz(move_dir)
    move_dir = obj_pos - robot.arm.get_ee_pose()[0]
    move_dir[2] += box_size * 2
    robot.arm.move_ee_xyz(move_dir)
    robot.arm.eetool.open()
    time.sleep(10)


if __name__ == '__main__':
    main()
