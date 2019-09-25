"""
A UR5e robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import numpy
from transforms3d.euler import euler2quat, euler2mat

from airobot.robot.robot import Robot
from airobot.sensor.camera.rgbd_cam import RGBDCamera

from airobot.utils import tcp_util
from airobot.utils import urscript_util

class RobotException(Exception):
    pass

class UR5eRobotReal(Robot):
    def __init__(self, cfgs, host):
        super(UR5eRobotReal, self).__init__(cfgs=cfgs)
        try:
            rospy.init_node('airobot', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [airobot] has already been initialized')
        
        self.camera = RGBDCamera(cfgs=cfgs)
        self._init_consts()

        self.host = host

        self.monitor = tcp_util.SecondaryMonitor(self.host)
        self.monitor.wait()  # make sure to make contact with the robot before doing anything

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

    def go_home(self):
        """
        Move the robot to a pre-defined home pose
        """
        # 6 joints for the arm, 7th joint for the gripper
        jnt_positions = [0., -1.5, 2.0, -2.05, -1.57]
        self.set_jpos(jnt_positions)

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
            # if len(position) == 6:
            #     gripper_pos = self.get_jpos(self.gripper_jnt_names[0])
            #     position.append(gripper_pos)
            
            if len(position) != 6:
                raise ValueError("position should contain 6 or 7 elements if" 
                                    "joint_name is not provided")

            # gripper_pos = position[-1] 
            # gripper_pos = min(self.gripper_open_angle, max(self.gripper_close_angle, gripper_pos))

            # position[-1] = gripper_pos

            target_pos = position 
        else:
            if joint_name is self.gripper_jnt_names:
                continue # don't do anything with gripper yet
            else:
                target_pos = position
                current_pos = self.get_jpos()
                rvl_jnt_idx = self.rvl_joint_names.index(joint_name)
                current_pos[rvl_jnt_idx] = target_pos 
        
        prog = "movej([%f, %f, %f, %f, %f, %f])" % target_pos*
        self.send_program(prog)
        if wait:
            success = self._wait_to_reach_jnt_goal(target_pos, joint_name=joint_name, mode="pos")

        return success 

    
    def set_jvel(self, velocity, acc=0.1, joint_name=None, wait=False, *args, **kwargs):
        """Set joint velocity in rad/s
        
        Args:
            velocity (list): list of target joint velocity values
            joint_name (str, optional): If not provided, velocity should be list and all joints
                will be turned on at specified velocity. Defaults to None.
            wait (bool, optional): [description]. Defaults to False.
        """
        velocity = copy.deepcopy(velocity)
        if joint_name is None:
            # if (len(velocity) == 6):
            #     gripper_vel = self.get_jvel(self.gripper_jnt_names[0])
            #     velocity.append(gripper_vel)

            if (len(velocity != 6)):
                raise ValueError("Velocity should contain 6 or 7 elements"
                                    "if the joint name is not provided")

            target_vel = velocity

        else:
            if joint_name in self.gripper_jnt_names:
                continue
            else:
                target_vel_joint = velocity
                target_vel = [0.0] * 7
                rvl_jnt_idx = self.rvl_joint_names.index(joint_name)
                target_vel[rvl_jnt_idx] = target_vel_joint

        prog = "speedj([%f, %f, %f, %f, %f, %f], a=%f)" % target_vel*, acc
        self.send_program(prog)

        #TODO see about getting back joint velocity info from the robot to use for success flag

    
    def set_ee_pose(self, pos, ori=None, acc=0.1, vel=0.05, *args, **kwargs):
        """Set cartesian space pose of end effector
        
        Args:
            pos (list): Desired x, y, z positions in the robot's base frame to move to
            ori (list, optional): Desired euler angle orientation of the end effector. Defaults to None.
            acc (float, optional): Acceleration of end effector during beginning of movement. Defaults to 0.1.
            vel (float, optional): Velocity of end effector during movement. Defaults to 0.05.
        
        Returns:
            [type]: [description]
        """
        if ori is None:
            ori = self.get_ee_pose[-1] # last index of return is the euler angle representation
        ee_pos = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2]]
        prog = "movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f, r=%f)" % ee_pos*, acc, vel
        self.send_program(prog)

        # TODO implement blocking version that checks whether or not we got there for success flag

    
    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        """Move end effector in straight line while maintaining a particular orientation
        
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
            self.set_ee_pose(waypoints[i, :].flatten().tolist(), ee_quat)
            time.sleep(0.01)
            # see to what extent this type of straight line end effector motion requires
            # breaking up the path like this

        # or instead of this ^ just
        # ee_pos[0] += delta_xyz[0]
        # ee_pos[1] += delta_xyz[1]
        # ee_pos[2] += delta_xyz[2]

        # self.set_ee_pose(ee_pos, ee_euler)
         


    def get_jpos(self, joint_name=None, wait=False):
        """Get current joint angles of the robot   
        
        Args:
            joint_name (str, optional): Defaults to None.

        Return:
            jpos (list): list of current joint positions in radians
        """
        jdata = self.monitor.get_joint_data(wait)
        jpos = [jdata["q_actual0"], jdata["q_actual1"], jdata["q_actual2"], jdata["q_actual3"], jdata["q_actual4"], jdata["q_actual5"]]
        return jpos 

    def get_jvel(self, joint_name=None, wait=False):
        """Get current joint angular velocities of the robot   
        
        Args:
            joint_name (str, optional): Defaults to None.

        Return:
            jvel (list): list of current joint angular velocities in radians/s
        """        
        jdata = self.monitor.get_joint_data(wait)
        jvel = [jdata["qd_actual0"], jdata["qd_actual1"], jdata["qd_actual2"], jdata["qd_actual3"], jdata["qd_actual4"], jdata["qd_actual5"]]
        return jvel


    def get_ee_pose(self, wait=False):
        """Get current cartesian pose of the end effector, in the robot's base frame
        
        Args:
            wait (bool, optional): [description]. Defaults to False.
        
        Returns:
            list: x, y, z position of the end effector
            list: quaternion representation of the end effector orientation
            list: rotation matrix representation of the end effector orientation
            list: euler angle representation of the end effector orientation
        """
        pose_data = self.monitor.get_cartesian_info(wait)
        if pose_data:
            pos = [pose_data["X"], pose_data["Y"], pose_data["Z"]]
            euler_ori = [pose_data["Rx"], pose_data["Ry"], pose_data["Rz"]]

            rot_mat = euler2mat(euler_ori)
            quat_ori = euler2quat(euler_ori)

        return list(pos), list(quat_ori), list(rot_mat), list(euler)


    def _get_dist(self, target, joints=False):
        """General distance function with flag for determining configuration space distance
        or cartesian space distance
        
        Args:
            target (list): Target configuration to compute distance to, from current
            joints (bool, optional): If True, compute distance in configuration space, otherwise in cartesian space. Defaults to False.
        
        Returns:
            float: Distance value, depending on which type of distance was requestedf
        """
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        """Get cartesian space linear distance between current and target end effector pose
        
        Args:
            target (list): Target pose for end effector
        
        Returns:
            float: euclidean distance between current end effector pose and target pose
        """
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist ** 0.5

    def _get_joints_dist(self, target):
        """Compute euclidean distance to target
        
        Args:
            target (list): Target joint angles to compute distance to, with respect to current joint angles
        
        Returns:
            list: euclidean distance between current and target joint configuration
        """
        joints = self.get_jpos(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
        return dist ** 0.5

    def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos', threshold=None, timeout=5):
        """Blocking function to ensure robot reaches goal configuration before doing anything else
        
        Args:
            goal (list): Target joint configuration
            joint_name ([type], optional): [description]. Defaults to None.
            mode (str, optional): Control mode ('pos' or 'vel'). Defaults to 'pos'.
            threshold (float, optional): Threshold to specify maximum allowable distance between current and target. Defaults to None.
            timeout (int, optional): Time in seconds before command returns as failed. Defaults to 5.
        
        Raises:
            RobotException: [description]
            RobotException: [description]
        """

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



    
