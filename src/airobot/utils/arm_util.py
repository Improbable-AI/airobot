import numbers
import time

import numpy as np
import airobot as ar
import airobot.utils.common as arutil


def wait_to_reach_jnt_goal(goal, get_func, joint_name=None,
                           get_func_derv=None, timeout=10.0, max_error=0.01):
    """
    Block the code to wait for the joint moving to the specified goal.
    The goal can be a desired velocity(s) or a desired position(s).
    if get_func returns the poisitions (velocities),
    then the get_func_derv should
    return the velocities (accelerations) (if provided).
    If the robot cannot reach the goal,
    providing get_func_derv can prevent the
    program from waiting until timeout.
    It uses the derivative information to make a faster judgement.

    Args:
        goal (float or list): goal positions or velocities
        get_func (function): name of the function with which we can get
            the current joint values
        joint_name (str): if it's none, all the actuated
            joints are compared.
            Otherwise, only the specified joint is compared
        get_func_derv (function): the name of the function with which we
            can get the derivative of the joint values
        timeout (float): maximum waiting time
        max_error (float): tolerance of error

    Returns:
        bool: if the goal is reached or not
    """
    success = False
    start_time = time.time()
    vel_stop_time = None
    if joint_name is not None:
        if not isinstance(goal, numbers.Number):
            raise ValueError('Only one goal should be '
                             'specified for a single joint!')
    while True:
        if time.time() - start_time > timeout:
            pt_str = 'Unable to move to joint goals (%s)' \
                     ' within %f s' % (str(goal),
                                       timeout)
            ar.log_error(pt_str)
            return success
        if reach_jnt_goal(goal, get_func, joint_name, max_error):
            success = True
            break
        if get_func_derv is not None:
            vel_threshold = 0.006
            jnt_vel = get_func_derv(joint_name)
            ar.log_info(jnt_vel)
            if np.max(np.abs(jnt_vel)) <= vel_threshold and vel_stop_time is None:
                vel_stop_time = time.time()
            elif np.max(np.abs(jnt_vel)) > vel_threshold:
                vel_stop_time = None
            if vel_stop_time is not None and time.time() - vel_stop_time > 1.5:
                pt_str = 'Unable to move to joint goals (%s)' % str(goal)
                ar.log_error(pt_str)
                return success
        time.sleep(0.001)
    return success


def reach_jnt_goal(goal, get_func, joint_name=None, max_error=0.01):
    """
    Check if the joint reached the goal or not.
    The goal can be a desired velocity(s) or a desired position(s).

    Args:
        goal (float or list): goal positions or velocities
        get_func (function): name of the function with which we can get
            the current joint values
        joint_name (str): if it's none, all the
            actuated joints are compared.
            Otherwise, only the specified joint is compared
        max_error (float): tolerance of error

    Returns:
        bool: if the goal is reached or not
    """
    goal = np.array(goal)
    new_jnt_val = get_func(joint_name)
    new_jnt_val = np.array(new_jnt_val)
    jnt_diff = new_jnt_val - goal
    error = np.max(np.abs(jnt_diff))
    if error < max_error:
        return True
    else:
        return False


def wait_to_reach_ee_goal(pos, ori, get_func, get_func_derv=None,
                          timeout=10.0, pos_tol=0.01, ori_tol=0.02):
    """
    Block the code to wait for the end effector to reach its
    specified goal pose (must be below both position and
    orientation threshold).

    Args:
        pos (list): goal position
        ori (list or np.ndarray): goal orientation. It can be:
            **quaternion** ([qx, qy, qz, qw], shape: :math:`[4]`)
            **rotation matrix** (shape: :math:`[3, 3]`)
            **euler angles** ([roll, pitch, yaw], shape: :math:`[3]`)
        get_func (function): name of the function with which we can get
            the end effector pose
        get_func_derv (function): the name of the function with which we
            can get end effector velocities
        timeout (float): maximum waiting time
        pos_tol (float): tolerance of position error
        ori_tol (float): tolerance of orientation error

    Returns:
        bool: If end effector reached goal or not
    """
    success = False
    start_time = time.time()
    vel_stop_time = None
    while True:
        if time.time() - start_time > timeout:
            pt_str = 'Unable to move to end effector position:' \
                     '%s and orientaion: %s within %f s' % \
                     (str(pos), str(ori), timeout)
            arutil.print_red(pt_str)
            return success
        if reach_ee_goal(pos, ori, get_func,
                         pos_tol=pos_tol,
                         ori_tol=ori_tol):
            success = True
            break
        if get_func_derv is not None:
            ee_pos_vel, ee_rot_vel = get_func_derv()
            ee_vel = np.concatenate((ee_pos_vel, ee_rot_vel))
            if np.max(np.abs(ee_vel)) < 0.001 and vel_stop_time is None:
                vel_stop_time = time.time()
            elif np.max(np.abs(ee_vel)) > 0.001:
                vel_stop_time = None
            if vel_stop_time is not None and time.time() - vel_stop_time > 1.5:
                pt_str = 'Unable to move to end effector pose\n' \
                         'pos: %s \n' \
                         'ori: %s ' % (str(pos), str(ori))
                arutil.print_red(pt_str)
                return success
        time.sleep(0.001)
    return success


def reach_ee_goal(pos, ori, get_func, pos_tol=0.01, ori_tol=0.02):
    """
    Check if end effector reached goal or not. Returns true
    if both position and orientation goals have been reached
    within specified tolerance

    Args:
        pos (list np.ndarray): goal position
        ori (list or np.ndarray): goal orientation. It can be:
            **quaternion** ([qx, qy, qz, qw], shape: :math:`[4]`)
            **rotation matrix** (shape: :math:`[3, 3]`)
            **euler angles** ([roll, pitch, yaw], shape: :math:`[3]`)
        get_func (function): name of the function with which we can get
            the current end effector pose
        pos_tol (float): tolerance of position error
        ori_tol (float): tolerance of orientation error


    Returns:
        bool: If goal pose is reached or not
    """
    if not isinstance(pos, np.ndarray):
        goal_pos = np.array(pos)
    else:
        goal_pos = pos
    if not isinstance(ori, np.ndarray):
        goal_ori = np.array(ori)
    else:
        goal_ori = ori

    if goal_ori.size == 3:
        goal_ori = arutil.euler2quat(goal_ori)
    elif goal_ori.shape == (3, 3):
        goal_ori = arutil.rot2quat(goal_ori)
    elif goal_ori.size != 4:
        raise TypeError('Orientation must be in one '
                        'of the following forms:'
                        'rotation matrix, euler angles, or quaternion')
    goal_ori = goal_ori.flatten()
    goal_pos = goal_pos.flatten()
    new_ee_pose = get_func()

    new_ee_pos = np.array(new_ee_pose[0])
    new_ee_quat = new_ee_pose[1]

    pos_diff = new_ee_pos.flatten() - goal_pos
    pos_error = np.max(np.abs(pos_diff))

    quat_diff = arutil.quat_multiply(arutil.quat_inverse(goal_ori),
                                     new_ee_quat)
    rot_similarity = np.abs(quat_diff[3])

    if pos_error < pos_tol and \
            rot_similarity > 1 - ori_tol:
        return True
    else:
        return False
