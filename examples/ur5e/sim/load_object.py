import time

import numpy as np

import airobot as ar
from airobot import Robot
from airobot.utils.common import euler2quat
from airobot.utils.pb_util import load_geom, load_urdf, get_body_state, remove_body


def main():
    """
    This function demonstrates how to load different kinds of
    objects and get state information of objects
    """
    np.set_printoptions(precision=3, suppress=True)
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()

    ori = euler2quat([0, 0, np.pi / 2])
    table_id = load_urdf('table/table.urdf', [1, 0, 0.4], ori, scaling=0.9)
    sphere_id = load_geom('sphere', size=0.05, mass=1,
                          base_pos=[1, 0, 1.0], rgba=[0, 1, 0, 1])
    box_id = load_geom('box', size=0.05, mass=1,
                       base_pos=[1, 0.12, 1.0], rgba=[1, 0, 0, 1])
    box_id2 = load_geom('box', size=[0.06, 0.02, 0.03], mass=1,
                        base_pos=[1.3, 0.12, 1.0], rgba=[0, 0, 1, 1])
    cylinder_id = load_geom('cylinder', size=[0.06, 0.08], mass=1,
                            base_pos=[0.8, -0.12, 1.0], rgba=[0, 1, 1, 1])
    duck_id = load_geom('mesh', mass=1, visualfile='duck.obj',
                        mesh_scale=0.1,
                        base_pos=[0.9, -0.4, 1.0],
                        rgba=[0.5, 0.2, 1, 1])
    pos, quat, lin_vel, ang_vel = get_body_state(cylinder_id)
    ar.log_info('Cylinder:')
    ar.log_info('         position: %s' % np.array2string(pos, precision=2))
    ar.log_info('         quaternion: %s' % np.array2string(quat, precision=2))
    ar.log_info('         linear vel: %s' % np.array2string(lin_vel, precision=2))
    ar.log_info('         angular vel: %s' % np.array2string(ang_vel, precision=2))
    ar.log_info('Removing sphere')
    res = remove_body(sphere_id)
    time.sleep(10)


if __name__ == '__main__':
    main()
