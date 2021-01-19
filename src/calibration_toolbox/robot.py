"""
UR Robot control and action with python-urx
Author: Hongtao Wu
Dec 22, 2019
"""

import socket
import select
import struct
import time
import os
import numpy as np
import itertools
import urx

from utils import make_rigid_transformation


class Robot(object):

    def __init__(self, tcp_host_ip='172.22.22.2', calibrate=False, acc=1, vel=1, gripper_on=True, go_home=True):
        '''
        UR5 Robot
        tcp_host_ip (string): tcp to connect to the robot
        calibrate (bool): to calibration or not
        '''

        print("Initializing the robot...")
        # Connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.robot = urx.Robot(tcp_host_ip)

        # Acceleration and veloctiy
        self.acc = acc
        self.vel = vel

        # Default home joint configuration
        self.home_config = (0.8367955088615417, -1.8993938604937952, 1.2887015342712402, -1.6408045927630823, -1.0915311018573206, -1.3117168585406702)
        
        # Sleep time between different frame
        self.sleep_time = 0.5

        # Initialize gripper
        if gripper_on:
            self.robot.set_tool_voltage(24)
            self.open_gripper()

        if go_home:
            self.go_home()

        #TODO: Initialize the camera

    
    def move_to(self, tool_position, tool_orientation=None, acc=None, vel=None):
        """Move robot to position and orientation
        Args:
        - tool_position (list): x, y, z
        - tool_orientation (list): rx, ry, rz (in the axis-angle UR5 tradition)
        """
        if acc is None:
            acc = self.acc

        if vel is None:
            vel = self.vel

        if tool_orientation is None:
            config = tuple(tool_position + [0, np.pi, 0])
            transform = self.robot.movel(config, acc, vel)
        else:
            config = tuple(tool_position + tool_orientation)
            transform = self.robot.movel(config, acc, vel)
        
        time.sleep(self.sleep_time)

        return transform
    

    def go_home(self):
        self.robot.movej(self.home_config, self.acc, self.vel)


    def get_pose(self):
        """Return a 4x4 numpy array rigid body transformation
        """

        xform = self.robot.get_pose()
        pos = xform.pos.array
        rotm = xform.orient.array

        return make_rigid_transformation(pos, rotm)

    def get_config(self):
        """Return the configuration of the robot ((6, ) list)
        """
        return self.robot.getj()


    def move_to_joint(self, joint_config, acc=None, vel=None):
        """Move in joint configuration
        Args:
        - joint_config (6, tuple): 6 joints for UR5 robot
        """
        if (not acc) or (not vel):
            self.robot.movej(joint_config, self.acc, self.vel)
        else: 
            self.robot.movej(joint_config, acc, vel)

    
    def close_gripper(self):
        """Using I/O to close the gripper (afag EU-20 UR)
        """
        self.robot.send_program("set_tool_digital_out(%s, %s)" % (0, True))
        time.sleep(0.5)
    
    
    def open_gripper(self):
        """Using I/O to open the gripper (afag EU-20 UR)
        """
        self.robot.send_program("set_tool_digital_out(%s, %s)" % (0, False))
        time.sleep(0.5)

    
    def disconnect(self):
        """Disconnect connection to the robot
        """
        self.robot.set_tool_voltage(0)
        self.robot.close()




# Test
if __name__ == "__main__":
    robot = Robot(acc=0.5, vel=0.5, gripper_on=False)
    robot.go_home()
    robot.disconnect()