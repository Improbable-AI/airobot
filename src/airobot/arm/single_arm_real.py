"""
A UR5e robot arm
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy

import PyKDL as kdl
import numpy as np
import rospy
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python import trac_ik

import airobot.utils.common as arutil
from airobot.arm.arm import ARM
from airobot.utils.ros_util import joints_to_kdl
from airobot.utils.ros_util import kdl_array_to_numpy
from airobot.utils.ros_util import kdl_frame_to_numpy


class SingleArmReal(ARM):
    def __init__(self, cfgs,
                 eetool_cfg=None):
        """
        This class contains methods that are common for most single-arm robots
        and do not depend on ROS

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
        """
        super(SingleArmReal, self).__init__(cfgs=cfgs, eetool_cfg=eetool_cfg)

        self._init_real_consts()

    def go_home(self):
        """
        Move the robot to a pre-defined home pose
        """
        self.set_jpos(self._home_position, wait=True)

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad)

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[6,]` if list, otherwise a single value)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        raise NotImplementedError

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
        raise NotImplementedError

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
        raise NotImplementedError

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
        raise NotImplementedError

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
        raise NotImplementedError

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
        raise NotImplementedError

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
        raise NotImplementedError

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control"

        Args:
            joint_angles (list or flattened np.ndarray): joint angles

        Returns:
            np.ndarray: jacobian (shape: :math:`[6, DOF]`)
        """
        q = kdl.JntArray(self.urdf_chain.getNrOfJoints())
        for i in range(q.rows()):
            q[i] = joint_angles[i]
        jac = kdl.Jacobian(self.urdf_chain.getNrOfJoints())
        fg = self.jac_solver.JntToJac(q, jac)
        if fg < 0:
            raise ValueError('KDL JntToJac error!')
        jac_np = kdl_array_to_numpy(jac)
        return jac_np

    def compute_fk_position(self, jpos, tgt_frame):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame (self.cfgs.ARM.ROBOT_BASE_FRAME). The desired frame
        must be in self.arm_link_names

        Args:
            jpos (list or flattened np.ndarray): joint angles
            tgt_frame (str): target link frame

        Returns:
            2-element tuple containing

            - np.ndarray: translational vector (shape: :math:`[3,]`)
            - np.ndarray: rotational matrix (shape: :math:`[3, 3]`)
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        jpos = jpos.flatten()
        if jpos.size != self.arm_dof:
            raise ValueError('Length of the joint angles '
                             'does not match the robot DOF')
        assert jpos.size == self.arm_dof
        kdl_jnt_angles = joints_to_kdl(jpos)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles,
                                          kdl_end_frame,
                                          idx)
        if fg < 0:
            raise ValueError('KDL Pos JntToCart error!')
        pose = kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].flatten()
        rot = pose[:3, :3]
        return pos, rot

    def compute_fk_velocity(self, jpos, jvel, tgt_frame):
        """
        Given joint_positions and joint velocities,
        compute the velocities of tgt_frame with respect
        to the base frame

        Args:
            jpos (list or flattened np.ndarray): joint positions
            jvel (list or flattened np.ndarray): joint velocities
            tgt_frame (str): target link frame

        Returns:
            np.ndarray: translational velocity and rotational velocity
            (vx, vy, vz, wx, wy, wz) (shape: :math:`[6,]`)
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        if isinstance(jvel, list):
            jvel = np.array(jvel)
        kdl_end_frame = kdl.FrameVel()
        kdl_jnt_angles = joints_to_kdl(jpos)
        kdl_jnt_vels = joints_to_kdl(jvel)
        kdl_jnt_qvels = kdl.JntArrayVel(kdl_jnt_angles, kdl_jnt_vels)
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self.fk_solver_vel.JntToCart(kdl_jnt_qvels,
                                          kdl_end_frame,
                                          idx)
        if fg < 0:
            raise ValueError('KDL Vel JntToCart error!')
        end_twist = kdl_end_frame.GetTwist()
        return np.array([end_twist[0], end_twist[1], end_twist[2],
                         end_twist[3], end_twist[4], end_twist[5]])

    def compute_ik(self, pos, ori=None, qinit=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector
        (self.cfgs.ARM.ROBOT_EE_FRAME)

        Args:
            pos (list or np.ndarray): position (shape: :math:`[3,]`)
            ori (list or np.ndarray): orientation. It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            qinit (list or np.ndarray): initial joint positions for numerical
                IK (shape: :math:`[6,]`)

        Returns:
            list: inverse kinematics solution (joint angles)
        """
        if ori is not None:
            ee_quat = arutil.to_quat(ori)
        else:
            ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()
        ori_x = ee_quat[0]
        ori_y = ee_quat[1]
        ori_z = ee_quat[2]
        ori_w = ee_quat[3]
        if qinit is None:
            qinit = self.get_jpos().tolist()
        elif isinstance(qinit, np.ndarray):
            qinit = qinit.flatten().tolist()
        pos_tol = self.cfgs.ARM.IK_POSITION_TOLERANCE
        ori_tol = self.cfgs.ARM.IK_ORIENTATION_TOLERANCE
        jnt_poss = self.num_ik_solver.get_ik(qinit,
                                             pos[0],
                                             pos[1],
                                             pos[2],
                                             ori_x,
                                             ori_y,
                                             ori_z,
                                             ori_w,
                                             pos_tol,
                                             pos_tol,
                                             pos_tol,
                                             ori_tol,
                                             ori_tol,
                                             ori_tol)
        if jnt_poss is None:
            return None
        return list(jnt_poss)

    def _init_real_consts(self):
        """
        Initialize constants
        """
        self._home_position = self.cfgs.ARM.HOME_POSITION

        robot_description = self.cfgs.ROBOT_DESCRIPTION
        urdf_string = rospy.get_param(robot_description)
        self.num_ik_solver = trac_ik.IK(self.cfgs.ARM.ROBOT_BASE_FRAME,
                                        self.cfgs.ARM.ROBOT_EE_FRAME,
                                        urdf_string=urdf_string)
        _, self.urdf_tree = treeFromParam(robot_description)
        base_frame = self.cfgs.ARM.ROBOT_BASE_FRAME
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        self.urdf_chain = self.urdf_tree.getChain(base_frame,
                                                  ee_frame)
        self.arm_jnt_names = self._get_kdl_joint_names()
        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_jnt_names)

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

        self.ee_link = self.cfgs.ARM.ROBOT_EE_FRAME

    def _get_kdl_link_names(self):
        """
        Internal method to get the link names from the KDL URDF chain

        Returns:
            list: List of link names
        """
        num_links = self.urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
        """
        Internal method to get the joint names from the KDL URDF chain

        Returns:
            list: List of joint names
        """
        num_links = self.urdf_chain.getNrOfSegments()
        num_joints = self.urdf_chain.getNrOfJoints()
        joint_names = []
        for i in range(num_links):
            link = self.urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)
