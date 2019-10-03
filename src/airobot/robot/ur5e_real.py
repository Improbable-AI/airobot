"""
A UR5e robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import sys
import threading
import time

import PyKDL as kdl
import moveit_commander
import numpy as np
import rospy
import tf
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction
from kdl_parser_py.urdf import treeFromParam
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_matrix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix
from trac_ik_python import trac_ik

from airobot.robot.robot import Robot
from airobot.end_effectors.robotiq2f140 import Robotiq2F140
from airobot.sensor.camera.rgbd_cam import RGBDCamera
from airobot.utils.common import clamp
from airobot.utils.common import joints_to_kdl
from airobot.utils.common import kdl_array_to_numpy
from airobot.utils.common import kdl_frame_to_numpy
from airobot.utils.common import print_red
from airobot.utils.moveit_util import MoveitScene
from airobot.utils.ros_util import get_tf_transform
from airobot.utils.ur_tcp_util import SecondaryMonitor


class UR5eRobotReal(Robot):
    def __init__(self, cfgs, use_cam=False, use_arm=True,
                 moveit_planner='RRTConnectkConfigDefault'):
        try:
            rospy.init_node('ur5e', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [ur5e] has already been initialized')
        if use_cam:
            self.camera = RGBDCamera(cfgs=cfgs)
        if use_arm:
            super(UR5eRobotReal, self).__init__(cfgs=cfgs)
            self.moveit_planner = moveit_planner
            self.gazebo_sim = rospy.get_param('sim')
            self._tcp_initialized = False
            self._init_consts()

            if not self.gazebo_sim:
                self.robot_ip = rospy.get_param('robot_ip')
                self.set_comm_mode()
                self._setup_pub_sub()
                self._set_tool_offset()

                # TODO temperalily disable tcp until we figure out
                # a better way to kill the program

                if self.use_urscript:
                    self._initialize_tcp_comm()
                tcp_monitor = None if not self.use_urscript else self.tcp_monitor
                self.gripper = Robotiq2F140(cfgs,
                                            self.use_urscript,
                                            tcp_monitor=tcp_monitor)

            else:
                raise NotImplementedError

    def __del__(self):
        self.stop()

    def stop(self):
        if hasattr(self, 'tcp_monitor'):
            self.tcp_monitor.close()

    def _initialize_tcp_comm(self):
        self.tcp_monitor = SecondaryMonitor(self.robot_ip)

        self.tcp_monitor.wait()  # make contact with robot before anything

    def set_comm_mode(self, use_urscript=False):
        """
        Method to set whether to use ros or urscript to control the real robot

        Arguments:
            use_urscript (bool): True we should use urscript
                False if we should use ros and moveit
        """
        self.use_urscript = use_urscript

    def _send_urscript(self, prog):
        """
        Method to send URScript program to the URScript ROS topic

        Args:
            prog (str): URScript program which will be sent and run on
                the UR5e machine

        """
        # TODO return the status info
        # such as if the robot gives any error,
        # the execution is successful or not
        self.urscript_pub.publish(prog)
        # TODO go back to TCP at some point, after debugging threading
        # self.tcp_monitor.send_program(prog)

    def output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant

        Args:
            msg (str): message to display

        Return:
            None
        """
        prog = 'textmsg(%s)' % msg
        self._send_urscript(prog)

    def _is_running(self):
        if hasattr(self, 'tcp_monitor'):
            return self.tcp_monitor.running
        else:
            raise ValueError('No TCP connection established')

    def go_home(self):
        """
        Move the robot to a pre-defined home pose
        """
        self.set_jpos(self._home_position, wait=True)
        # self.gripper.open_gripper()

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot

        Args:
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move
            wait (bool): whether position command should be blocking or non
                blocking

        Return:
            success (bool): whether command was completed successfully or not
        """
        position = copy.deepcopy(position)
        success = False

        if joint_name is None:
            if len(position) != 6:
                raise ValueError('position should contain 6 elements if'
                                 'joint_name is not provided')
            tgt_pos = position
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = position
        if self.use_urscript:
            prog = 'movej([%f, %f, %f, %f, %f, %f])' % (tgt_pos[0],
                                                        tgt_pos[1],
                                                        tgt_pos[2],
                                                        tgt_pos[3],
                                                        tgt_pos[4],
                                                        tgt_pos[5])
            self._send_urscript(prog)

            if wait:
                success = self._wait_to_reach_jnt_goal(
                    tgt_pos,
                    joint_name=joint_name,
                    mode='pos')
        else:
            self.moveit_group.set_joint_value_target(tgt_pos)
            success = self.moveit_group.go(tgt_pos, wait=wait)

        return success

    def set_jvel(self, velocity, acc=0.1, joint_name=None, wait=False,
                 *args, **kwargs):
        """
        Set joint velocity in rad/s

        Args:
            velocity (list): list of target joint velocity values
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): [description]. Defaults to False.
        """
        velocity = copy.deepcopy(velocity)
        success = False

        if joint_name is None:
            if len(velocity) != 6:
                raise ValueError('Velocity should contain 6 elements'
                                 'if the joint name is not provided')
            tgt_vel = velocity
        else:
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_vel = [0.0] * len(self.arm_jnt_names)
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_vel[arm_jnt_idx] = velocity
        if self.use_urscript:
            prog = 'speedj([%f, %f, %f, %f, %f, %f], a=%f)' % (tgt_vel[0],
                                                               tgt_vel[1],
                                                               tgt_vel[2],
                                                               tgt_vel[3],
                                                               tgt_vel[4],
                                                               tgt_vel[5],
                                                               acc)
            self._send_urscript(prog)
        else:
            self._pub_joint_vel(tgt_vel)

        if wait:
            success = self._wait_to_reach_jnt_goal(tgt_vel,
                                                   joint_name=joint_name,
                                                   mode='vel')

        return success

    def set_ee_pose(self, pos, ori=None, acc=0.1, vel=0.05, wait=True,
                    ik_first=False, *args, **kwargs):
        """
        Set cartesian space pose of end effector

        Args:
            pos (list): Desired x, y, z positions in the robot's base frame to
                move to
            ori (list, optional): Desired euler angle orientation (roll, pitch, yaw)
                or quaternion ([x, y, z, w]) of the end effector. It Defaults to None.
            acc (float, optional): Acceleration of end effector during
                beginning of movement. Defaults to 0.1.
            vel (float, optional): Velocity of end effector during movement.
                Defaults to 0.05.

        Returns:
            bool: success or failure to move the robot to the goal pose
        """
        success = False
        if ori is None:
            pose = self.get_ee_pose()[-1]  # last index of return is euler anglexs
            quat = pose[1]
            euler = pose[-1]
        elif len(ori) == 4:
            quat = ori
            # assume incoming orientation is quaternion
            euler = euler_from_quaternion(quat)
        elif len(ori) == 3:
            euler = ori
            quat = quaternion_from_euler(*euler)
        else:
            raise ValueError('Orientaion should be quaternion or'
                             'euler angles')

        if self.use_urscript:
            if ik_first:
                jnt_pos = self.compute_ik(pos, quat)  # ik can handle quaternion
                # use movej instead of movel
                success = self.set_jpos(jnt_pos, wait=wait)
            else:
                ee_pos = [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]
                prog = 'movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f, r=%f)' % (
                    ee_pos[0],
                    ee_pos[1],
                    ee_pos[2],
                    ee_pos[3],
                    ee_pos[4],
                    ee_pos[5],
                    acc,
                    vel,
                    0.0)
                self._send_urscript(prog)
                if wait:
                    success = self._wait_to_reach_ee_goal(pos, quat)
        else:
            pose = self.moveit_group.get_current_pose()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            self.moveit_group.set_pose_target(pose)
            success = self.moveit_group.go(wait=True)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, wait=True, *args, **kwargs):
        """Move end effector in straight line while maintaining orientation

        Args:
            delta_xyz (list): Goal change in x, y, z position of end effector
            eef_step (float, optional): [description]. Defaults to 0.005.
        """
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        if self.use_urscript:
            ee_pos[0] += delta_xyz[0]
            ee_pos[1] += delta_xyz[1]
            ee_pos[2] += delta_xyz[2]
            success = self.set_ee_pose(ee_pos, ee_euler, wait=wait,
                                       ik_first=False)
        else:
            cur_pos = np.array(ee_pos)
            cur_quat = ee_quat
            delta_xyz = np.array(delta_xyz)
            end_pos = cur_pos + delta_xyz
            moveit_waypoints = []
            wpose = self.moveit_group.get_current_pose().pose
            wpose.position.x = cur_pos[0]
            wpose.position.y = cur_pos[1]
            wpose.position.z = cur_pos[2]
            wpose.orientation.x = cur_quat[0]
            wpose.orientation.y = cur_quat[1]
            wpose.orientation.z = cur_quat[2]
            wpose.orientation.w = cur_quat[3]
            moveit_waypoints.append(copy.deepcopy(wpose))

            wpose.position.x = end_pos[0]
            wpose.position.y = end_pos[1]
            wpose.position.z = end_pos[2]
            wpose.orientation.x = cur_quat[0]
            wpose.orientation.y = cur_quat[1]
            wpose.orientation.z = cur_quat[2]
            wpose.orientation.w = cur_quat[3]
            moveit_waypoints.append(copy.deepcopy(wpose))

            # delta_xyz = np.array(delta_xyz)
            # path_len = np.linalg.norm(delta_xyz)
            # num_pts = int(np.ceil(path_len / float(eef_step)))
            # if num_pts <= 1:
            #     num_pts = 2
            # waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
            # waypoints = cur_pos + waypoints_sp / float(path_len) * delta_xyz
            # moveit_waypoints = []
            # wpose = self.moveit_group.get_current_pose().pose
            # for i in range(waypoints.shape[0]):
            #     wpose.position.x = waypoints[i, 0]
            #     wpose.position.y = waypoints[i, 1]
            #     wpose.position.z = waypoints[i, 2]
            #     wpose.orientation.x = cur_quat[0]
            #     wpose.orientation.y = cur_quat[1]
            #     wpose.orientation.z = cur_quat[2]
            #     wpose.orientation.w = cur_quat[3]
            #     moveit_waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = self.moveit_group.compute_cartesian_path(
                moveit_waypoints,  # waypoints to follow
                eef_step,  # eef_step
                0.0)  # jump_threshold
            success = self.moveit_group.execute(plan, wait=wait)
        return success

    def get_jpos(self, joint_name=None):
        self._j_state_lock.acquire()
        if joint_name is not None:
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] not recognized!' % joint_name)
            jpos = self._j_pos[joint_name]
        else:
            jpos = []
            for joint in self.arm_jnt_names:
                jpos.append(self._j_pos[joint])
        self._j_state_lock.release()
        return jpos

    def get_jvel(self, joint_name=None):
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
        """Get current cartesian pose of the EE, in the robot's base frame

        Returns:
            list: x, y, z position of the EE (shape: [3])
            list: quaternion representation ([x, y, z, w]) of the EE orientation (shape: [4])
            list: rotation matrix representation of the EE orientation
                (shape: [3, 3])
            list: euler angle representation of the EE orientation (roll,
                pitch, yaw with static reference frame) (shape: [3])
        """
        pos, quat = get_tf_transform(self.tf_listener,
                                     self.cfgs.ROBOT_BASE_FRAME,
                                     self.cfgs.ROBOT_EE_FRAME)
        rot_mat = quaternion_matrix(quat)[:3, :3].tolist()
        euler_ori = list(euler_from_quaternion(quat))
        return pos, quat, rot_mat, euler_ori

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgba/depth images

        Args:
            get_rgb (bool): return rgb image if True, None otherwise
            get_depth (bool): return depth image if True, None otherwise

        Returns:
            np.ndarray: rgba and depth images
        """
        return self.camera.get_images(get_rgb, get_depth)

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control"

        Args:
            joint_angles (list or flattened np.ndarray): joint angles

        Returns:
            jacobian (list, shape: [6, 6])
        """
        q = kdl.JntArray(self.urdf_chain.getNrOfJoints())
        for i in range(q.rows()):
            q[i] = joint_angles[i]
        jac = kdl.Jacobian(self.urdf_chain.getNrOfJoints())
        fg = self.jac_solver.JntToJac(q, jac)
        assert fg == 0, 'KDL JntToJac error!'
        jac_np = kdl_array_to_numpy(jac)
        return jac_np.tolist()

    def compute_fk_position(self, jpos, tgt_frame):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame (self.cfgs.ROBOT_BASE_FRAME). The desired frame
        must be in self.arm_link_names

        Args:
            jpos (list or flattened np.ndarray): joint angles
            tgt_frame (str): target link frame

        Returns:
            translational vector (list, shape: [3,])
            and rotational matrix (list, shape: [3, 3])
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
        if fg == 0:
            raise ValueError('KDL Pos JntToCart error!')
        pose = kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].flatten().tolist()
        rot = pose[:3, :3].tolist()
        return pos, rot

    def compute_fk_velocity(self, jpos, jvel, tgt_frame):
        """
        Given joint_positions and joint velocities,
        compute the velocities of des_frame with respect
        to the base frame

        Args:
            jpos (list or flattened np.ndarray): joint positions
            jvel (list or flattened np.ndarray): joint velocities
            tgt_frame (str): target link frame

        Returns:
            translational and rotational
                 velocities (vx, vy, vz, wx, wy, wz)
                 (list, shape: [6,])
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
        if fg == 0:
            raise ValueError('KDL Vel JntToCart error!')
        end_twist = kdl_end_frame.GetTwist()
        return [end_twist[0], end_twist[1], end_twist[2],
                end_twist[3], end_twist[4], end_twist[5]]

    def compute_ik(self, pos, ori=None, qinit=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector
        (self.cfgs.ROBOT_EE_FRAME)

        Args:
            pos (list): position
            ori (list): orientation. It can be euler angles
                (roll, pitch, yaw) or quaternion. If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            qinit (list): initial joint positions for numerical IK

        Returns:
            inverse kinematics solution (joint angles, list)
        """
        if ori is not None:
            if len(ori) == 3:
                # [roll, pitch, yaw]
                ori = quaternion_from_euler(*ori)
            if len(ori) != 4:
                raise ValueError('Orientation should be either '
                                 'euler angles or quaternion')
            ori_x = ori[0]
            ori_y = ori[1]
            ori_z = ori[2]
            ori_w = ori[3]
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
        pos_tol = self.cfgs.IK_POSITION_TOLERANCE
        ori_tol = self.cfgs.IK_ORIENTATION_TOLERANCE
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

    def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Block the code to wait for the joint moving to the specified goal.
        The goal can be a desired velocity(s) or a desired position(s).
        Max waiting time is self.cfgs.TIMEOUT_LIMIT

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the actuated
                joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pos' or 'vel'

        Returns:
            if the goal is reached or not
        """
        success = False
        start_time = time.time()
        while True:
            if not self._is_running():
                raise RuntimeError("Robot stopped")

            if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                pt_str = 'Unable to move to joint goals [mode: %s] (%s)' \
                         ' within %f s' % (mode, str(goal),
                                           self.cfgs.TIMEOUT_LIMIT)
                print_red(pt_str)
                return success
            if self._reach_jnt_goal(goal, joint_name, mode=mode):
                success = True
                break
            time.sleep(0.001)
        return success

    def _reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Check if the joint reached the goal or not.
        The goal can be a desired velocity(s) or a desired position(s).

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the
                actuated joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pose' or 'vel'

        Returns:
            if the goal is reached or not
        """
        goal = np.array(goal)
        if mode == 'pos':
            new_jnt_val = self.get_jpos(joint_name)
        elif mode == 'vel':
            new_jnt_val = self.get_jvel(joint_name)
        else:
            raise ValueError('Only pos and vel modes are supported!')
        new_jnt_val = np.array(new_jnt_val)
        jnt_diff = new_jnt_val - goal
        error = np.max(np.abs(jnt_diff))
        if error < self.cfgs.MAX_JOINT_ERROR:
            return True
        else:
            return False

    def _wait_to_reach_ee_goal(self, pos, ori):
        """
        Block the code to wait for the end effector to reach its
        specified goal pose (must be below both position and
        orientation threshold). Max waiting time is
        self.cfgs.TIMEOUT_LIMIT

        Args:
            pos (list): goal position
            ori (list or np.ndarray): goal orientation. It can be:
                quaternion ([qx, qy, qz, qw])
                rotation matrix ([3, 3])
                euler angles ([roll, pitch, yaw])

        Returns:
            bool: If end effector reached goal or not
        """
        success = False
        start_time = time.time()
        while True:
            if not self._is_running():
                raise RuntimeError("Robot stopped")

            if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                pt_str = 'Unable to move to end effector position:' \
                         '%s and orientaion: %s within %f s' % (str(pos), str(ori),
                                                                self.cfgs.TIMEOUT_LIMIT)
                print_red(pt_str)
                return success
            if self._reach_ee_goal(pos, ori):
                success = True
                break
            time.sleep(0.001)
        return success

    def _reach_ee_goal(self, pos, ori):
        """
        Check if end effector reached goal or not. Returns true
        if both position and orientation goals have been reached
        within specified tolerance

        Args:
            pos (list np.ndarray): goal position
            ori (list or np.ndarray): goal orientation. It can be:
                quaternion ([qx, qy, qz, qw])
                rotation matrix ([3, 3])
                euler angles ([roll, pitch, yaw])

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
            goal_ori = quaternion_from_euler(goal_ori[0],
                                             goal_ori[1],
                                             goal_ori[2])
            goal_ori = np.array(goal_ori)
        elif goal_ori.size == 9:
            rot = np.eye(4)
            rot[:3, :3] = goal_ori
            goal_ori = quaternion_from_matrix(rot)
            goal_ori = np.array(goal_ori)
        elif goal_ori.size != 4:
            raise TypeError('Orientation must be in one '
                            'of the following forms:'
                            'rotation matrix, euler angles, or quaternion')
        goal_ori = goal_ori.flatten()
        goal_pos = goal_pos.flatten()
        new_ee_pose = self.get_ee_pose()

        new_ee_pos = np.array(new_ee_pose[0])
        new_ee_quat = new_ee_pose[1]

        pos_diff = new_ee_pos.flatten() - goal_pos
        pos_error = np.max(np.abs(pos_diff))

        quat_diff = quaternion_multiply(quaternion_inverse(goal_ori),
                                        new_ee_quat)
        rot_similarity = np.abs(quat_diff[3])

        if pos_error < self.cfgs.MAX_EE_POSITION_ERROR and \
                rot_similarity > 1 - self.cfgs.MAX_EE_ORIENTATION_ERROR:
            return True
        else:
            return False

    def _init_consts(self):
        """
        Initialize constants
        """
        self._home_position = self.cfgs.HOME_POSITION

        robot_description = self.cfgs.ROBOT_DESCRIPTION
        urdf_string = rospy.get_param(robot_description)
        self.num_ik_solver = trac_ik.IK(self.cfgs.ROBOT_BASE_FRAME,
                                        self.cfgs.ROBOT_EE_FRAME,
                                        urdf_string=urdf_string)
        _, self.urdf_tree = treeFromParam(robot_description)

        self.urdf_chain = self.urdf_tree.getChain(self.cfgs.ROBOT_BASE_FRAME,
                                                  self.cfgs.ROBOT_EE_FRAME)
        self.arm_jnt_names = self._get_kdl_joint_names()
        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_jnt_names)
        self.gripper_tip_pos, self.gripper_tip_ori = self._get_tip_transform()

        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_group = MoveGroupCommander(self.cfgs.MOVEGROUP_NAME)
        self.moveit_group.set_planner_id(self.moveit_planner)
        self.moveit_scene = MoveitScene()
        self.scale_moveit_motion(vel_scale=0.2, acc_scale=0.2)

        # add a virtual base support frame of the real robot:
        ur_base_name = 'ur_base'
        ur_base_attached = False
        for i in range(2):
            self.moveit_scene.add_static_obj(ur_base_name,
                                             [0, 0, -0.5],
                                             [0, 0, 0, 1],
                                             size=[0.25, 0.50, 1.0],
                                             obj_type='box',
                                             ref_frame='/base_link')
            time.sleep(1)
            obj_dict, obj_adict = self.moveit_scene.get_objects()
            if ur_base_name in obj_dict.keys():
                ur_base_attached = True
                break
        if not ur_base_attached:
            print_red('Fail to add the UR base support as a collision object. '
                      'Be careful when you use moveit to plan the path! You can'
                      'try again to add the base manually.')
        self.traj_follower_client = SimpleActionClient(self.cfgs.TRAJ_FOLLOW_CLIENT_NS,
                                                       FollowJointTrajectoryAction)
        self.traj_follower_client.wait_for_server(timeout=rospy.Duration(self.cfgs.TIMEOUT_LIMIT))

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

        self.gripper_jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self.gripper_jnt_names_set = set(self.gripper_jnt_names)

        self.ee_link = self.cfgs.ROBOT_EE_FRAME

        self._j_pos = dict()
        self._j_vel = dict()
        self._j_torq = dict()
        self._j_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(self.cfgs.ROSTOPIC_JOINT_STATES, JointState,
                         self._callback_joint_states)

        # https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
        self._max_torques = [150, 150, 150, 28, 28, 28]
        # a random value for robotiq joints
        self._max_torques.append(20)
        time.sleep(1.5)

    def scale_moveit_motion(self, vel_scale=1.0, acc_scale=1.0):
        vel_scale = clamp(vel_scale, 0.0, 1.0)
        acc_scale = clamp(acc_scale, 0.0, 1.0)
        self.moveit_group.set_max_velocity_scaling_factor(vel_scale)
        self.moveit_group.set_max_acceleration_scaling_factor(acc_scale)

    def _callback_joint_states(self, msg):
        """
        ROS subscriber callback for arm joint states

        Args:
            msg (sensor_msgs/JointState): Contains message published in topic

        Returns:

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

    def _get_kdl_link_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
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

    def _get_tip_transform(self):
        gripper_tip_id = self.arm_link_names.index(self.cfgs.ROBOT_EE_FRAME)
        gripper_tip_link = self.urdf_chain.getSegment(gripper_tip_id)
        gripper_tip_tf = kdl_frame_to_numpy(gripper_tip_link.getFrameToTip())
        gripper_tip_pos = gripper_tip_tf[:3, 3].flatten()
        gripper_tip_rot_mat = np.eye(4)
        gripper_tip_rot_mat[:3, :3] = gripper_tip_tf[:3, :3]
        gripper_tip_euler = euler_from_matrix(gripper_tip_rot_mat)
        return list(gripper_tip_pos), list(gripper_tip_euler)

    def _set_tool_offset(self):
        tool_offset_prog = 'set_tcp(p[%f, %f, %f, %f, %f, %f])' % (
            self.gripper_tip_pos[0],
            self.gripper_tip_pos[1],
            self.gripper_tip_pos[2],
            self.gripper_tip_ori[0],
            self.gripper_tip_ori[1],
            self.gripper_tip_ori[2]
        )

        self._send_urscript(tool_offset_prog)

    def _pub_joint_vel(self, velocity):
        """
        Received joint velocity command, puts it in a ROS trajectory
        msg, and uses internal publisher to send to /joint_speed topic
        on the real robot

        Args:
            velocity (list): List of desired joint velocities
                (length and order have already been checked)
        """
        goal_speed_msg = JointTrajectory()
        goal_speed_msg.points.append(
            JointTrajectoryPoint(
                velocities=velocity))
        self.joint_vel_pub.publish(goal_speed_msg)

    def _setup_pub_sub(self):
        """
        Initialize all the publishers and subscribers used internally
        """
        # for publishing joint speed to real robot
        self.joint_vel_pub = rospy.Publisher(
            self.cfgs.JOINT_SPEED_TOPIC,
            JointTrajectory,
            queue_size=10
        )

        self.urscript_pub = rospy.Publisher(
            self.cfgs.URSCRIPT_TOPIC,
            String,
            queue_size=10
        )
