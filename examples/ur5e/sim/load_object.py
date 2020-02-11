import time

import numpy as np
from airobot import Robot
from airobot import log_info
from airobot.utils.common import euler2quat


def main():
    """
    This function demonstrates how to load different kinds of
    objects and get state information of objects.
    """
    np.set_printoptions(precision=3, suppress=True)
    robot = Robot('ur5e_stick')
    robot.arm.go_home()

    ori = euler2quat([0, 0, np.pi / 2])
    robot.pb_client.load_urdf('table/table.urdf',
                              [1, 0, 0.4],
                              ori,
                              scaling=0.9)
    sphere_id = robot.pb_client.load_geom('sphere',
                                          size=0.05,
                                          mass=1,
                                          base_pos=[1, 0, 1.0],
                                          rgba=[0, 1, 0, 1])
    robot.pb_client.load_geom('box',
                              size=0.05,
                              mass=1,
                              base_pos=[1, 0.12, 1.0],
                              rgba=[1, 0, 0, 1])
    robot.pb_client.load_geom('box',
                              size=[0.06, 0.02, 0.03],
                              mass=1,
                              base_pos=[1.3, 0.12, 1.0],
                              rgba=[0, 0, 1, 1])
    cylinder_id = robot.pb_client.load_geom('cylinder',
                                            size=[0.06, 0.08],
                                            mass=1,
                                            base_pos=[0.8, -0.12, 1.0],
                                            rgba=[0, 1, 1, 1])
    duck_id = robot.pb_client.load_geom('mesh',
                                        mass=1,
                                        visualfile='duck.obj',
                                        mesh_scale=0.1,
                                        base_pos=[0.9, -0.4, 1.0],
                                        rgba=[0.5, 0.2, 1, 1])
    pos, quat, lin_vel, ang_vel = robot.pb_client.get_body_state(cylinder_id)
    log_info('Cylinder:')
    log_info('         position: %s' % np.array2string(pos,
                                                       precision=2))
    log_info('         quaternion: %s' % np.array2string(quat,
                                                         precision=2))
    log_info('         linear vel: %s' % np.array2string(lin_vel,
                                                         precision=2))
    log_info('         angular vel: %s' % np.array2string(ang_vel,
                                                          precision=2))
    log_info('Removing sphere')
    robot.pb_client.remove_body(sphere_id)
    time.sleep(2)
    log_info('Reset duck')
    robot.pb_client.reset_body(duck_id, base_pos=[0.9, -0.4, 1.0],
                               base_quat=[0, 0, 0, 1],
                               lin_vel=[0, 2, 0],
                               ang_vel=[0, 0, 2])
    time.sleep(10)


if __name__ == '__main__':
    main()
