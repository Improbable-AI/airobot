from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import importlib


class ARM(object):
    """
    Base class for robots
    """

    def __init__(self, cfgs, eetool_cfg=None):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the robot
        """
        self.cfgs = cfgs
        if cfgs.HAS_EETOOL:
            if eetool_cfg is None:
                eetool_cfg = {}
            mod = importlib.import_module('airobot.ee_tool')
            eetool_calss = getattr(mod, cfgs.EETOOL.CLASS)
            self.eetool = eetool_calss(cfgs, **eetool_cfg)

    def go_home(self):
        """
        Move the robot arm to a pre-defined home pose
        """
        raise NotImplementedError

    def set_jpos(self, position, joint_name=None, *args, **kwargs):
        """
        Move the arm to the specified joint position(s).

        Args:
            position (float or list or flattened np.ndarray): desired joint position(s)
            joint_name (str): If not provided, position should be a list
                and all the actuated joints will be moved to the specified
                positions. If provided, only the specified joint will
                be moved to the desired joint position
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def set_jvel(self, velocity, joint_name=None, *args, **kwargs):
        """
        Move the arm with the specified joint velocity(ies).

        Args:
            velocity (float or list or flattened np.ndarray): desired joint velocity(ies)
            joint_name (str): If not provided, velocity should be a list
                and all the actuated joints will be moved in the specified
                velocities. If provided, only the specified joint will
                be moved in the desired joint velocity
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def set_jtorq(self, torque, joint_name=None, *args, **kwargs):
        """
        Apply torque(s) to the joint(s)

        Args:
            torque (float or list or flattened np.ndarray): torque value(s) for the joint(s)
            joint_name (str): specify the joint on which the torque is applied.
                If it's not provided(None), it will apply the torques on
                the actuated joints on the arm. Otherwise,
                only the specified joint will be applied with
                the given torque.
            wait (bool): whether to block the code and wait
                for the action to complete

        Returns:
            bool: A boolean variable representing if the action is successful at
            the moment when the function exits

        """
        raise NotImplementedError

    def set_ee_pose(self, pos=None, ori=None, *args, **kwargs):
        """
        Move the end effector to the specifed pose
        Args:
            pos (list or np.ndarray): position
            ori (list or np.ndarray): orientation. It can be either quaternion (length is 4)
                or euler angles ([roll, pitch, yaw])

        Returns:
            bool: A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """
        Move the end-effector in a straight line without changing the
        orientation

        Args:
            delta_xyz (list or np.ndarray): movement in x, y, z directions
            eef_step (float): interpolation interval along delta_xyz. Interpolate
                a point every eef_step distance between the two end points

        Returns:
            bool: A boolean variable representing if the action is successful at
            the moment when the function exits
        """
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s)

        Args:
            joint_name (str): If it's None, it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            float: joint position given joint_name
            or
            list: joint positions if joint_name is None
        """
        raise NotImplementedError

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies)

        Args:
            joint_name (str): If it's None, it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint velocity of the specified joint

        Returns:
            float: joint velocity given joint_name
            or
            list: joint velocities if joint_name is None
        """
        raise NotImplementedError

    def get_jtorq(self, joint_name=None):
        """
        Return the joint torque(s)

        Args:
            joint_name (str): If it's None, it will return joint torques
                of all the actuated joints. Otherwise, it will
                return the joint torque of the specified joint

        Returns:
            float: joint velocity given joint_name
            or
            list: joint velocities if joint_name is None
        """
        raise NotImplementedError

    def get_ee_pose(self):
        """
        Return the end effector pose

        Returns:
            np.ndarray: x, y, z position of the EE (shape: [3])
            np.ndarray: quaternion representation ([x, y, z, w]) of the EE
                orientation (shape: [4])
            np.ndarray: rotation matrix representation of the EE orientation
                (shape: [3, 3])
            np.ndarray: euler angle representation of the EE orientation (roll,
                pitch, yaw with static reference frame) (shape: [3])
        """
        raise NotImplementedError

    def compute_ik(self, pos, ori=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector

        Args:
            pos (list or np.ndarray): position (shape: [3,])
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: [4,]),
                or quaternion ([qx, qy, qz, qw], shape: [4,]),
                or rotation matrix (shape: [3, 3]). If it's None,
                the solver will use the current end effector
                orientation as the target orientation

        Returns:
            list: inverse kinematics solution (joint angles)
        """
        raise NotImplementedError
    #
    # def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
    #     """
    #     Block the code to wait for the joint moving to the specified goal.
    #     The goal can be a desired velocity(s) or a desired position(s).
    #     Max waiting time is self.cfgs.ARM.TIMEOUT_LIMIT
    #
    #     Args:
    #         goal (float or list): goal positions or velocities
    #         joint_name (str): if it's none, all the actuated
    #             joints are compared.
    #             Otherwise, only the specified joint is compared
    #         mode (str): 'pos' or 'vel'
    #
    #     Returns:
    #         bool: if the goal is reached or not
    #     """
    #     success = False
    #     start_time = time.time()
    #     vel_stop_time = None
    #     if joint_name is not None and not isinstance(goal, float):
    #         raise ValueError('Only one goal should be specified for a single joint!')
    #     while True:
    #         if time.time() - start_time > self.cfgs.ARM.TIMEOUT_LIMIT:
    #             pt_str = 'Unable to move to joint goals [mode: %s] (%s)' \
    #                      ' within %f s' % (mode, str(goal),
    #                                        self.cfgs.ARM.TIMEOUT_LIMIT)
    #             print_red(pt_str)
    #             return success
    #         reach_goal = self._reach_jnt_goal(goal, joint_name, mode=mode)
    #         if reach_goal:
    #             success = True
    #             break
    #         if mode == 'pos':
    #             jnt_vel = self.get_jvel(joint_name)
    #             if np.max(np.abs(jnt_vel)) < 0.001 and vel_stop_time is None:
    #                 vel_stop_time = time.time()
    #             elif np.max(np.abs(jnt_vel)) > 0.001:
    #                 vel_stop_time = None
    #             if vel_stop_time is not None and time.time() - vel_stop_time > 1.5:
    #                 pt_str = 'Unable to move to joint goals [mode: %s] (%s)' % (mode, str(goal))
    #                 print_red(pt_str)
    #                 return success
    #         time.sleep(0.001)
    #     return success
    #
    # def _reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
    #     """
    #     Check if the joint reached the goal or not.
    #     The goal can be a desired velocity(s) or a desired position(s).
    #
    #     Args:
    #         goal (float or list): goal positions or velocities
    #         joint_name (str): if it's none, all the
    #             actuated joints are compared.
    #             Otherwise, only the specified joint is compared
    #         mode (str): 'pose' or 'vel'
    #
    #     Returns:
    #         bool: if the goal is reached or not
    #     """
    #     goal = np.array(goal)
    #     if mode == 'pos':
    #         new_jnt_val = self.get_jpos(joint_name)
    #     elif mode == 'vel':
    #         new_jnt_val = self.get_jvel(joint_name)
    #     else:
    #         raise ValueError('Only pos and vel modes are supported!')
    #     new_jnt_val = np.array(new_jnt_val)
    #     jnt_diff = new_jnt_val - goal
    #     error = np.max(np.abs(jnt_diff))
    #     if error < self.cfgs.ARM.MAX_JOINT_ERROR:
    #         return True
    #     else:
    #         return False
    #
    # def _wait_to_reach_ee_goal(self, pos, ori):
    #     """
    #     Block the code to wait for the end effector to reach its
    #     specified goal pose (must be below both position and
    #     orientation threshold). Max waiting time is
    #     self.cfgs.ARM.TIMEOUT_LIMIT
    #
    #     Args:
    #         pos (list): goal position
    #         ori (list or np.ndarray): goal orientation. It can be:
    #             quaternion ([qx, qy, qz, qw])
    #             rotation matrix ([3, 3])
    #             euler angles ([roll, pitch, yaw])
    #
    #     Returns:
    #         bool: If end effector reached goal or not
    #     """
    #     success = False
    #     start_time = time.time()
    #     while True:
    #         if time.time() - start_time > self.cfgs.ARM.TIMEOUT_LIMIT:
    #             pt_str = 'Unable to move to end effector position:' \
    #                      '%s and orientaion: %s within %f s' % \
    #                      (str(pos), str(ori), self.cfgs.ARM.TIMEOUT_LIMIT)
    #             print_red(pt_str)
    #             return success
    #         if self._reach_ee_goal(pos, ori):
    #             success = True
    #             break
    #         time.sleep(0.001)
    #     return success
    #
    # def _reach_ee_goal(self, pos, ori):
    #     """
    #     Check if end effector reached goal or not. Returns true
    #     if both position and orientation goals have been reached
    #     within specified tolerance
    #
    #     Args:
    #         pos (list np.ndarray): goal position
    #         ori (list or np.ndarray): goal orientation. It can be:
    #             quaternion ([qx, qy, qz, qw])
    #             rotation matrix ([3, 3])
    #             euler angles ([roll, pitch, yaw])
    #
    #     Returns:
    #         bool: If goal pose is reached or not
    #     """
    #     if not isinstance(pos, np.ndarray):
    #         goal_pos = np.array(pos)
    #     else:
    #         goal_pos = pos
    #     if not isinstance(ori, np.ndarray):
    #         goal_ori = np.array(ori)
    #     else:
    #         goal_ori = ori
    #
    #     if goal_ori.size == 3:
    #         goal_ori = arutil.euler2quat(goal_ori)
    #     elif goal_ori.shape == (3, 3):
    #         goal_ori = arutil.rot2quat(goal_ori)
    #     elif goal_ori.size != 4:
    #         raise TypeError('Orientation must be in one '
    #                         'of the following forms:'
    #                         'rotation matrix, euler angles, or quaternion')
    #     goal_ori = goal_ori.flatten()
    #     goal_pos = goal_pos.flatten()
    #     new_ee_pose = self.get_ee_pose()
    #
    #     new_ee_pos = np.array(new_ee_pose[0])
    #     new_ee_quat = new_ee_pose[1]
    #
    #     pos_diff = new_ee_pos.flatten() - goal_pos
    #     pos_error = np.max(np.abs(pos_diff))
    #
    #     quat_diff = arutil.quat_multiply(arutil.quat_inverse(goal_ori),
    #                                      new_ee_quat)
    #     rot_similarity = np.abs(quat_diff[3])
    #
    #     if pos_error < self.cfgs.ARM.MAX_EE_POSITION_ERROR and \
    #             rot_similarity > 1 - self.cfgs.ARM.MAX_EE_ORIENTATION_ERROR:
    #         return True
    #     else:
    #         return False
