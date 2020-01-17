#! /usr/bin/env python

"""
Tools for creating URScript messages, for communicating with the real UR
robots over TCP/IP by creating URScript programs and sending them for
execution on the robot's system

built off of urscript.py, part of python-urx library
(https://github.com/SintefManufacturing/python-urx)
"""

import airobot as ar


class URScript(object):
    """
    Class for creating urscript programs to send to the
    UR5 controller
    """

    def __init__(self):
        """
        Constructor, each urscript has a header and a program
        that runs on the contoller
        """
        # The program is code inside the myProg() method
        self._program = ""

    def __call__(self):
        """
        Create a string formatted properly as a urscript program
        upon call
        """
        if self._program == "":
            ar.log_debug("urscript program is empty")
            return ""

        # Construct the program
        myprog = """def myProg():{}\nend""".format(self._program)

        # Construct the full script
        script = ""
        script = "{}{}".format(script, myprog)
        return script

    def reset(self):
        """
        Reset the urscript to empty
        """
        self._program = ""

    def _add_line_to_program(self, new_line):
        """
        Add new line to program inside myProg()

        Args:
            new_line (str): Line to add to myProg() urscript program
        """
        self._program = "{}\n\t{}".format(self._program, new_line)

    def constrain_unsigned_char(self, value):
        """
        Ensure that unsigned char values are constrained
        to between 0 and 255.

        Args:
            value (int): Value to ensure is between 0 and 255
        """
        assert (isinstance(value, int))
        if value < 0:
            value = 0
        elif value > 255:
            value = 255
        return value

    def sleep(self, value):
        """
        Add a sleep command to urscript program, sleep for a
        specified amount of time

        Args:
            value (float): Amount of time in seconds for program
                to sleep
        """
        msg = "sleep({})".format(value)
        self._add_line_to_program(msg)

    def socket_open(self, socket_host, socket_port, socket_name):
        """
        Add a open socket command to urscript program with specified
        host, port, and name

        Args:
            socket_host (str): Host address
            socket_port (int): Port to open
            socket_name (str): Name of the socket to use when interacting
                with the socket after it is opened
        """
        msg = "socket_open(\"{}\",{},\"{}\")".format(socket_host,
                                                     socket_port,
                                                     socket_name)
        self._add_line_to_program(msg)

    def socket_close(self, socket_name):
        """
        Add a close socket command to urscript program.

        Args:
            socket_name (str): Name of socket to close (must be same as
                name of socket that was opened previously)
        """
        msg = "socket_close(\"{}\")".format(socket_name)
        self._add_line_to_program(msg)

    def socket_get_var(self, var, socket_name):
        """
        Add a command to the program with communicates over a socket
        connection to get the value of a variable

        Args:
            var (str): Name of the variable to obtain the value of
            socket_name (str): Which socket connection to use for
                getting the value (must be same as name of socket that was
                opened previously)
        """
        msg = "socket_get_var(\"{}\",\"{}\")".format(var, socket_name)
        self._add_line_to_program(msg)
        self.sync()

    def socket_set_var(self, var, value, socket_name):
        """
        Add a command to the program with communicates over a socket
        connection to set the value of a variable

        Args:
            var (str): Name of the variable to obtain the value of
            value (int): Value to set the variable to
            socket_name (str): Which socket connection to use for
                getting the value (must be same as name of socket that was
                opened previously)
        """
        msg = "socket_set_var(\"{}\",{},\"{}\")".format(
            var,
            value,
            socket_name)
        self._add_line_to_program(msg)
        self.sync()

    def sync(self):
        """
        Add a sync command to the myProg() urscript program
        """
        msg = "sync()"
        self._add_line_to_program(msg)


class Robotiq2F140URScript(URScript):
    """
    Class for creating Robotiq 2F140 specific URScript
    messages to send to the UR robot, for setting gripper
    related variables

    Args:
        socket_host (str): gripper IP address used by the UR controller
        socket_port (int): gripper communication port used by the UR controller
        socket_name (str): name of the socket connection
    """

    def __init__(self,
                 socket_host,
                 socket_port,
                 socket_name):
        self._socket_host = socket_host
        self._socket_port = socket_port
        self._socket_name = socket_name
        super(Robotiq2F140URScript, self).__init__()

        # reset gripper connection
        self.socket_close(self._socket_name)
        self.socket_open(
            self._socket_host,
            self._socket_port,
            self._socket_name
        )

    def set_activate(self):
        """
        Activate the gripper, by setting some internal
        variables on the UR controller to 1
        """
        self.socket_set_var('ACT', 1, self._socket_name)
        self.socket_set_var('GTO', 1, self._socket_name)

    def set_gripper_position(self, position):
        """
        Control the gripper position by setting internal
        position variable to desired position value on
        UR controller

        Args:
            position (int): Position value, ranges from 0-255
        """
        position = self.constrain_unsigned_char(position)
        self.socket_set_var('POS', position, self._socket_name)

    def set_gripper_speed(self, speed):
        """
        Set what speed the gripper should move

        Args:
            speed (int): Desired gripper speed, ranges from 0-255
        """
        speed = self.constrain_unsigned_char(speed)
        self.socket_set_var('SPE', speed, self._socket_name)

    def set_gripper_force(self, force):
        """
        Set maximum gripper force

        Args:
            force (int): Desired maximum gripper force, ranges
                from 0-255
        """
        force = self.constrain_unsigned_char(force)
        self.socket_set_var('FOR', force, self._socket_name)
