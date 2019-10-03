import rospy

from std_msgs.msg import String

from airobot.end_effectors.ee import EndEffector
from airobot.utils.urscript_util import Robotiq2F140URScript
from airobot.utils.common import clamp


class Robotiq2F140(EndEffector):
    """
    Class for interfacing with Robotiq 2F140 gripper when
    it is attached to UR5e arm. Communication with the gripper
    is either through ROS over through a TCP/IP socket
    """
    def __init__(self, cfgs, tcp_monitor=None):
        """
        Constructor for Robotiq2F140 class

        Args:
            cfgs (YACS CfgNode): configurations for the gripper
            use_tcp (bool, optional): Whether to use TCP/IP 
                monitor to communicate or not. Defaults to False.
            tcp_monitor (SecondaryMonitor): Interface to TCP
                socket, if using TCP/IP. Defaults to None
        """
        super(Robotiq2F140, self).__init__(cfgs=cfgs)
        self.tcp_monitor = tcp_monitor

        use_tcp = False if self.tcp_monitor is None else True

        self._tcp_initialized = False
        self._ros_initialized = False
        self.set_comm_mode(use_tcp)
        if not self.use_tcp:
            self._initialize_ros_comm()
        else:
            self._initialize_tcp_comm()

    def __del__(self):
        """
        Descructor
        """
        self.close_tcp()

    def _get_new_urscript(self):
        """
        Internal method used to create an empty URScript
        program, which is filled with URScript commands and
        eventually sent to the robot over one of the communication
        interfaces
        """
        urscript = Robotiq2F140URScript(
            socket_host=self.cfgs.GRIPPER.SOCKET_HOST,
            socket_port=self.cfgs.GRIPPER.SOCKET_PORT,
            socket_name=self.cfgs.GRIPPER.SOCKET_NAME)

        urscript.sleep(0.1)
        return urscript

    def activate(self):
        """
        Method to activate the gripper
        """
        urscript = self._get_new_urscript()

        # urscript.set_gripper_force(self.cfgs.GRIPPER.DEFAULT_FORCE)
        # urscript.set_gripper_speed(self.cfgs.GRIPPER.DEFAULT_SPEED)
        urscript.set_activate()

        urscript.sleep(0.1)

        if not self.use_tcp:
            self.pub_command.publish(urscript())
        else:
            self.tcp_monitor.send_program(urscript())

    def set_position(self, position):
        """
        Set the gripper position. Function internally maps
        values from API position range to URScript position
        range

        Args:
            position (float): Desired gripper position
        """
        urscript = self._get_new_urscript()

        position = clamp(
            position,
            self.cfgs.GRIPPER.OPEN_ANGLE,
            self.cfgs.GRIPPER.CLOSE_ANGLE
        )
        position = int(position * self.cfgs.GRIPPER.POSITION_SCALING)

        urscript.set_gripper_position(position)
        urscript.sleep(2.0)

        if not self.use_tcp:
            self.pub_command.publish(urscript())
        else:
            self.tcp_monitor.send_program(urscript())

    def open(self):
        """
        Open gripper
        """
        self.set_position(self.cfgs.GRIPPER.OPEN_ANGLE)

    def close(self):
        """
        Close gripper
        """
        self.set_position(self.cfgs.GRIPPER.CLOSE_ANGLE)

    def close_tcp(self):
        if self._tcp_initialized:
            # close the tcp communication
            pass

    def _initialize_ros_comm(self):
        """
        Set up the internal publisher to send gripper command
        URScript programs to the robot thorugh ROS
        """
        self.pub_command = rospy.Publisher(
            self.cfgs.GRIPPER.COMMAND_TOPIC,
            String,
            queue_size=10)
        self._ros_initialized = True
        rospy.sleep(2.0)

    def _initialize_tcp_comm(self):
        """
        Set internal variables to know we are using TCP/IP

        Raises:
            ValueError: If we did not successfully obtain the
                TCP monitor interface, raise an error
        """
        if self.tcp_monitor is None:
            raise ValueError('TCP monitor has not been initialized!')
        self._tcp_initialized = True

    def set_comm_mode(self, use_tcp=False):
        """
        Set what communication mode to use, default is ROS

        Args:
            use_tcp (bool, optional): Whether to use TCP/IP
                monitor or not, Defaults to False
        """
        self.use_tcp = use_tcp
