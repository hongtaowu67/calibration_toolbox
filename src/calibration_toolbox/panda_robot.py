# Class to interact with Panda

import numpy as np

import rospy
import tf2_ros

import std_msgs.msg

from aruco_calib.srv import *

class PandaRobot(object):
    """
    class used to interact with the Panda robot
    """
    def __init__(self, base_frame="panda_link0", ee_frame="panda_hand", go_home=True, load_gripper=False):
        
        rospy.loginfo("Start setting up the panda robot")

        # Home configuration
        self.home_config = [0.0, -np.pi/4, 0.0, -2*np.pi/3, 0.0, np.pi/3, np.pi/4]

        # Sleep time between different frame
        self.sleep_time = 0.5

        # Set up MoveToJoint client
        self.setupMoveToJointClient()

        if go_home:
            # Move robot home
            self.goHome()

    def setupMoveToJointClient(self):
        rospy.loginfo("Start setting up MoveToJoint client...")
        rospy.wait_for_service("move_to_joint")
        self.move_joint_client = rospy.ServiceProxy("move_to_joint", MoveToJoint)
        rospy.loginfo("Finish setting up MoveToJoint client...")

    def moveToJointPosition(self, joint_config):
        """
        Call the move_to_joint client to move the robot
        
        joint_config (list of 7): joint configuration
        """
        ros_joint_config = std_msgs.msg.Float64MultiArray()
        ros_joint_config.data = joint_config
        try:
            resp = self.move_joint_client(ros_joint_config)
        except rospy.ServiceException as e:
            rospy.login("Service call failed: %s" % e)

    def goHome(self):
        """
        Move to Home configuration
        """
        self.moveToJointPosition(self.home_config)

    # def moveToPose(self, pos, orn):
    #     if orn is None:
    #         config = tuple(pos + [0, np.pi, 0])
    #         transform = self.moveToJointPosition(config)
    #     else:
    #         config = tuple(pos + orn)
    #         transform = self.moveToJointPosition(config)
        
    #     rospy.sleep(self.sleep_time)

    #     return transform