import rospy
from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal

from airobot.ee_tool.ee import EndEffectorTool
from airobot.utils.common import clamp
from airobot.utils.urscript_util import Robotiq2F140URScript


class Robotiq2F140Real(EndEffectorTool):
    """
    Class for interfacing with Robotiq 2F140 gripper when
    it is attached to UR5e arm. Communication with the gripper
    is either through ROS over through a TCP/IP socket
    """

    def __init__(self, cfgs):
        """
        Constructor for Robotiq2F140 class

        Args:
            cfgs (YACS CfgNode): configurations for the gripper
        """
        super(Robotiq2F140Real, self).__init__(cfgs=cfgs)
        self.jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self.gazebo_sim = rospy.get_param('sim')

        self.jnt_names_set = set(self.jnt_names)

        self._ros_initialized = False
        self._initialize_ros_comm()

    def activate(self):
        """
        Method to activate the gripper
        """
        if not self.gazebo_sim:
            urscript = self._get_new_urscript()

            urscript.set_activate()
            urscript.set_gripper_speed(255)  # move at max speed

            urscript.sleep(0.1)

            self.pub_command.publish(urscript())
        rospy.sleep(0.2)

    def set_pos(self, pos):
        """
        Set the gripper position. Function internally maps
        values from API position range to URScript position
        range

        Args:
            pos (float): Desired gripper position
        """
        pos = clamp(
            pos,
            self.cfgs.EETOOL.OPEN_ANGLE,
            self.cfgs.EETOOL.CLOSE_ANGLE
        )
        if not self.gazebo_sim:
            urscript = self._get_new_urscript()

            pos = int(pos * self.cfgs.EETOOL.POSITION_SCALING)

            urscript.set_gripper_position(pos)
            urscript.sleep(0.1)
            
            gripper_cmd = urscript()
        else:
            gripper_cmd = GripperCommandActionGoal()
            gripper_cmd.goal.command.position = pos
        self.pub_command.publish(gripper_cmd)

    def open(self):
        """
        Open gripper
        """
        self.set_pos(self.cfgs.EETOOL.OPEN_ANGLE)

    def close(self):
        """
        Close gripper
        """
        self.set_pos(self.cfgs.EETOOL.CLOSE_ANGLE)

    def _get_new_urscript(self):
        """
        Internal method used to create an empty URScript
        program, which is filled with URScript commands and
        eventually sent to the robot over one of the communication
        interfaces
        """
        urscript = Robotiq2F140URScript(
            socket_host=self.cfgs.EETOOL.SOCKET_HOST,
            socket_port=self.cfgs.EETOOL.SOCKET_PORT,
            socket_name=self.cfgs.EETOOL.SOCKET_NAME)

        urscript.sleep(0.1)
        return urscript

    def _initialize_ros_comm(self):
        """
        Set up the internal publisher to send gripper command
        URScript programs to the robot thorugh ROS
        """
        if self.gazebo_sim:
            self.pub_command = rospy.Publisher(
                self.cfgs.EETOOL.GAZEBO_COMMAND_TOPIC,
                GripperCommandActionGoal,
                queue_size=10)
        else:
            self.pub_command = rospy.Publisher(
                self.cfgs.EETOOL.COMMAND_TOPIC,
                String,
                queue_size=10)
        self._ros_initialized = True
        rospy.sleep(2.0)
