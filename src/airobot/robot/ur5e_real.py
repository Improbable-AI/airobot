"""
A UR5e robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import numpy
import transforms3d

from airobot.robot.robot import Robot
from airobot.sensor.camera.rgbd_cam import RGBDCamera

from airobot.utils import tcp_util
from airobot.utils import urscript_util


class UR5eRobotReal(Robot):
    def __init__(self, cfgs, host):
        super(UR5eRobotReal, self).__init__(cfgs=cfgs)
        try:
            rospy.init_node('airobot', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [airobot] has already been initialized')
        self.camera = RGBDCamera(cfgs=cfgs)
        
        self.host = host

        self.monitor = tcp_util.SecondaryMonitor(self.host)
        self.monitor.wait() # make sure to make contact with the robot before doing anything

    def send_program(self, prog):
        """
        Method to send URScript program to the TCP/IP monitor

        Args: 
            prog (str): URScript program which will be sent and run on the UR5e machine

        Return:
            None 
        """
        self.monitor.send_program(prog)

    def output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant 

        Args:
            msg (str): message to display

        Return: 
            None
        """
        prog = "textmsg(%s)" % msg
        self.send_program(prog)


    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot

        Args: 
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If 
                provided, only specified joint will move
            wait (bool): whether position command should be blocking or non blocking

        Return:
            success (bool): whether command was completed successfully or not
        """
        position = copy.deepcopy(position)
        success = False

        if joint_name is None:
            if len(position) == 6:
                gripper_pos = self.get_jpos(self.gripper_jnt_names[0])
                position.append(gripper_pos)
            
            if len(position) != 7:
                raise ValueError("position should contain 6 or 7 elements if" 
                                    "joint_name is not provided")

            gripper_pos = position[-1] 
            gripper_pos = min(self.gripper_open_angle, max(self.gripper_close_angle, gripper_pos))

            position[-1] = gripper_pos

            target_pos = position 
            prog = self._format_move_command("movej", joints, acc, vel)
            self.send_program(prog)
        else:
            if joint_name is self.gripper_jnt_names:

            else:
                target_pos = position
                rvl_jnt_idx = self.rvl_joint_names.index(joint_name)
                jnt_id = self.jnt_to_id[joint_name]

        if wait:
            success = self._wait_to_reach_jnt_goal(target_pos, joint_name=joint_name, mode="pos")

        return success 

    
    def set_jvel(self, velocity, joint_name=None, wait=False, *args, **kwargs):
        """[summary]
        
        Args:
            velocity ([type]): [description]
            joint_name ([type], optional): [description]. Defaults to None.
            wait (bool, optional): [description]. Defaults to False.
        """

    
    def set_ee_pose(self, pos, ori=None, acc=0.1, vel=0.05, *args, **kwargs):
        if ori is None:
            ori = self.get_ee_pose[-1]
        ee_pos = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2]]
        prog = "movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f, r=%f)" % *ee_pos, acc, vel
        self.send_program(prog)

    
    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """[summary]
        
        Args:
            delta_xyz (list): Goal change in x, y, z position of end effector
            eef_step (float, optional): [description]. Defaults to 0.005.
        """
        success = True 
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        current_pos = np.array(ee_pos)
        delta_xyz = np.array(delta_xyz)
        path_len = np.linalg.norm(delta_xyz)
        num_pts = int(np.ceil(path_len / float(eef_step)))
        if num_pts <= 1:
            num_pts = 2
        
        waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
        waypoints = cur_pos + waypoints_sp / float(path_len) * delta_xyz

        for i in range(waypoints.shape[0]):
            target_j

        ee_pose[0]
        ee_pose[1]
        ee_pose[2]

        # send movel command to robot
        return 


    def get_jpos(self, joint_name=None, wait=False):
        """[summary]
        
        Args:
            joint_name ([type], optional): [description]. Defaults to None.

        Return:
            jpos ([list]): list of current joint positions in radians
        """
        jdata = self.monitor.get_joint_data(wait)
        jpos = [jdata["q_actual0"], jdata["q_actual1", jdata["q_actual2"], jdata["q_actual3"], jdata["q_actual4"], jdata["q_actual5"]]
        return jpos 


    def get_jvel(self, joint_name=None):


    def get_ee_pose(self, wait=False):
        pose_data = self.monitor.get_cartesian_info(wait)
        if pose_data:
            pos = [pose_data["X"], pose_data["Y"], pose_data["Z"]]
            euler_ori = [pose_data["Rx"], pose_data["Ry"], pose_data["Rz"]]

            rot_mat = transforms3d.euler.euler2mat(euler_ori)
            quat_ori = transforms3d.euler.euler2quat(euler_ori)

        return list(pos), list(quat_ori), list(rot_mat), list(euler)


    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist ** 0.5

    def _get_joints_dist(self, target):
        joints = self.get_jpos(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
        return dist ** 0.5

    def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos', threshold=None, timeout=5):

        start_dist = self._get_joint_dist(goal)
        if threshold is None:
            threshold = start_dist * 0.8
            if threshold < 0.001:
                threshold = 0.001
        
        count = 0
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")

            dist = self._get_joints_dist(goal)

            if not self.monitor.is_program_running():
                if dist < threshold:
                    return

                count += 1
                if count > timeout * 10:
                    raise RobotException("Goal not reached, timeout reached")
            else:
                count = 0


    def _format_move_command(self, ur_command, tpose, acc, vel, radius=0, prefix=""):

        move_command = "{}({}[{},{},{},{},{},{}], a={}, v={}, r={})" % ur_command, prefix, *tpose

    def _build_jnt_id(self):
        """
        Build the mapping from the joint name to joint index
        """
        self.jnt_to_id = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            jnt_name = info[1].decode('UTF-8')
            self.jnt_to_id[jnt_name] = info[0]

        # joint ids for the actuators
        # only the first joint in the gripper is the actuator
        # in the simulation
        act_joint_names = self.arm_jnt_names + [self.gripper_jnt_names[0]]
        self.actuator_ids = [self.jnt_to_id[jnt] for jnt in act_joint_names]
        # joint ids for all joints
        self.rvl_jnt_ids = [
            self.jnt_to_id[jnt] for jnt in self.rvl_joint_names
        ]

        self.ee_link_id = self.jnt_to_id[self.ee_link]
        self.arm_jnt_ids = [self.jnt_to_id[jnt] for jnt in self.arm_jnt_names]
        self.gripper_jnt_ids = [
            self.jnt_to_id[jnt] for jnt in self.gripper_jnt_names
        ]


    def _init_consts(self):
        """
        Initialize constants
        """
        # joint damping for inverse kinematics
        self._ik_jd = 0.05
        self._thread_sleep = 0.001

        self.arm_jnt_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_dof = len(self.arm_jnt_names)
        self.gripper_jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self.gripper_close_angle = 0.7
        self.gripper_open_angle = 0
        self.gripper_jnt_names_set = set(self.gripper_jnt_names)
        self.rvl_joint_names = self.arm_jnt_names + self.gripper_jnt_names
        self._ik_jds = [self._ik_jd] * len(self.rvl_joint_names)
        self.ee_link = 'wrist_3_link-tool0_fixed_joint'

        # https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
        self._max_torques = [150, 150, 150, 28, 28, 28]
        # a random value for robotiq joints
        self._max_torques.append(20)
        self.camera = PyBulletCamera(p, self.cfgs)



    
