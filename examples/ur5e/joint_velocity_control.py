import math
import time

import airobot as ar


def sin_wave(t, f, A):
    return A * math.cos(2 * math.pi * f * t)


def main():
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True})
    robot.go_home()

    A = 0.2
    f = 0.4
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        vels = [sin_wave(elapsed_time, f, A)] * robot.arm_dof
        robot.set_jvel(vels)
        time.sleep(0.01)


if __name__ == '__main__':
    main()
