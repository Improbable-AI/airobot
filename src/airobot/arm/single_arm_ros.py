"""
A UR5e robot arm
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numbers
import sys
import threading

import moveit_commander
import numpy as np
import rospy
import tf
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState

import airobot.utils.common as arutil
from airobot.arm.single_arm_real import SingleArmReal
from airobot.utils.moveit_util import MoveitScene
from airobot.utils.moveit_util import moveit_cartesian_path
from airobot.utils.ros_util import get_tf_transform


class SingleArmROS(SingleArmReal):
    def __init__(self, cfgs,
                 moveit_planner='RRTstarkConfigDefault',
                 eetool_cfg=None):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            moveit_planner (str): motion planning algorithm
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
        """
        super(SingleArmROS, self).__init__(cfgs=cfgs, eetool_cfg=eetool_cfg)

        self.moveit_planner = moveit_planner
        self.gazebo_sim = rospy.get_param('sim')
        self._init_ros_consts()

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad)

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[DOF,]` if list, otherwise a single value)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        position = copy.deepcopy(position)
        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('position should contain %d elements if '
                                 'joint_name is not provided' % self.arm_dof)
            tgt_pos = position
        else:
            if not isinstance(position, numbers.Number):
                raise TypeError('position should be individual float value'
                                ' if joint_name is provided')
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = position
        self.moveit_group.set_joint_value_target(tgt_pos)
        success = self.moveit_group.go(tgt_pos, wait=wait)

        return success

    def set_jvel(self, velocity, joint_name=None, wait=False,
                 *args, **kwargs):
        """
        Set joint velocity command to the robot (units in rad/s)

        Args:
            velocity (float or list or flattened np.ndarray): list of target
                joint velocity value(s)
                (shape: :math:`[6,]` if list, otherwise a single value)
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): If True, block until robot reaches
                desired joint velocity value(s). Defaults to False.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        raise NotImplementedError

    def set_ee_pose(self, pos=None, ori=None, wait=True, *args, **kwargs):
        """
        Set cartesian space pose of end effector

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`)
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            wait (bool): wait until the motion completes

        Returns:
            bool: Returns True is robot successfully moves to goal pose
        """
        if ori is None and pos is None:
            return True
        if ori is None:
            pose = self.get_ee_pose()  # last index is euler angles
            quat = pose[1]
        else:
            quat = arutil.to_quat(ori)
        if pos is None:
            pose = self.get_ee_pose()
            pos = pose[0]

        pose = self.moveit_group.get_current_pose()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.moveit_group.set_pose_target(pose)
        success = self.moveit_group.go(wait=wait)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, wait=True,
                    *args, **kwargs):
        """
        Move end effector in straight line while maintaining orientation

        Args:
            delta_xyz (list or np.ndarray): Goal change in x, y, z position of
                end effector
            eef_step (float, optional): Discretization step in cartesian space
                for computing waypoints along the path. Defaults to 0.005 (m).
            wait (bool, optional): True if robot should not do anything else
                until this goal is reached, or the robot times out.
                Defaults to True.

        Returns:
            bool: True if robot successfully reached the goal pose
        """
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        plan = moveit_cartesian_path(ee_pos,
                                     ee_quat,
                                     delta_xyz,
                                     self.moveit_group,
                                     eef_step)
        success = self.moveit_group.execute(plan, wait=wait)
        return success

    def get_jpos(self, joint_name=None):
        """
        Gets the current joint position of the robot. Gets the value
        from the internally updated dictionary that subscribes to the ROS
        topic /joint_states

        Args:
            joint_name (str, optional): If it's None,
                it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            One of the following

            - float: joint position given joint_name
            - list: joint positions if joint_name is None
              (shape: :math:`[DOF]`)

        """
        self._j_state_lock.acquire()
        if joint_name is not None:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] '
                                'not recognized!' % joint_name)
            jpos = self._j_pos[joint_name]
        else:
            jpos = []
            for joint in self.arm_jnt_names:
                jpos.append(self._j_pos[joint])
        self._j_state_lock.release()
        return jpos

    def get_jvel(self, joint_name=None):
        """
        Gets the current joint angular velocities of the robot. Gets the value
        from the internally updated dictionary that subscribes to the ROS
        topic /joint_states

        Args:
            joint_name (str, optional): If it's None,
                it will return joint velocities
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint

        Returns:
            One of the following

            - float: joint velocity given joint_name
            - list: joint velocities if joint_name is None
              (shape: :math:`[DOF]`)
        """
        self._j_state_lock.acquire()
        if joint_name is not None:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] not recognized!' % joint_name)
            jvel = self._j_vel[joint_name]
        else:
            jvel = []
            for joint in self.arm_jnt_names:
                jvel.append(self._j_vel[joint])
        self._j_state_lock.release()
        return jvel

    def get_ee_pose(self):
        """
        Get current cartesian pose of the EE, in the robot's base frame,
        using ROS subscriber to the tf tree topic

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3]`)
            - np.ndarray: quaternion representation ([x, y, z, w]) of the EE
              orientation (shape: :math:`[4]`)
            - np.ndarray: rotation matrix representation of the EE orientation
              (shape: :math:`[3, 3]`)
            - np.ndarray: euler angle representation of the EE orientation
              (roll, pitch, yaw with static reference frame)
              (shape: :math:`[3]`)
        """
        pos, quat = get_tf_transform(self.tf_listener,
                                     self.cfgs.ARM.ROBOT_BASE_FRAME,
                                     self.cfgs.ARM.ROBOT_EE_FRAME)
        rot_mat = arutil.quat2rot(quat)
        euler_ori = arutil.quat2euler(quat)
        return np.array(pos), np.array(quat), rot_mat, euler_ori

    def get_ee_vel(self):
        """
        Return the end effector's velocity

        Returns:
            2-element tuple containing

            - np.ndarray: translational velocity (vx, vy, vz)
              (shape: :math:`[3,]`)
            - np.ndarray: rotational velocity
              (wx, wy, wz) (shape: :math:`[3,]`)
        """
        jpos = self.get_jpos()
        jvel = self.get_jvel()
        ee_vel = self.compute_fk_velocity(jpos, jvel,
                                          self.cfgs.ARM.ROBOT_EE_FRAME)
        return ee_vel[:3], ee_vel[3:]

    def scale_motion(self, vel_scale=1.0, acc_scale=1.0):
        """
        Sets the maximum velocity and acceleration for the robot
        motion. Specified as a fraction
        from 0.0 - 1.0 of the maximum velocity and acceleration
        specified in the MoveIt joint limits configuration file.

        Args:
            vel_scale (float): velocity scale, Defaults to 1.0
            acc_scale (float): acceleration scale, Defaults to 1.0
        """
        vel_scale = arutil.clamp(vel_scale, 0.0, 1.0)
        acc_scale = arutil.clamp(acc_scale, 0.0, 1.0)
        self.moveit_group.set_max_velocity_scaling_factor(vel_scale)
        self.moveit_group.set_max_acceleration_scaling_factor(acc_scale)
        self._motion_vel = self.max_vel * vel_scale
        self._motion_acc = self.max_acc * acc_scale

    def _init_ros_consts(self):
        """
        Initialize constants
        """
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_group = MoveGroupCommander(self.cfgs.ARM.MOVEGROUP_NAME)
        self.moveit_group.set_planner_id(self.moveit_planner)
        self.moveit_group.set_planning_time(1.0)
        self.moveit_scene = MoveitScene()

        # read the joint limit (max velocity and acceleration) from the
        # moveit configuration file
        jnt_params = []
        max_vels = []
        max_accs = []
        for arm_jnt in self.arm_jnt_names:
            jnt_param = self.cfgs.ROBOT_DESCRIPTION + \
                        '_planning/joint_limits/' + arm_jnt
            jnt_params.append(copy.deepcopy(jnt_param))
            max_vels.append(rospy.get_param(jnt_param + '/max_velocity'))
            max_accs.append(rospy.get_param(jnt_param + '/max_acceleration'))
        self.max_vel = np.min(max_vels)
        self.max_acc = np.min(max_accs)

        self._j_pos = dict()
        self._j_vel = dict()
        self._j_torq = dict()
        self._j_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.cfgs.ARM.ROSTOPIC_JOINT_STATES, JointState,
                         self._callback_joint_states)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint states

        Args:
            msg (sensor_msgs/JointState): Contains message published in topic
        """
        self._j_state_lock.acquire()
        for idx, name in enumerate(msg.name):
            if name in self.arm_jnt_names:
                if idx < len(msg.position):
                    self._j_pos[name] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._j_vel[name] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._j_torq[name] = msg.effort[idx]
        self._j_state_lock.release()
