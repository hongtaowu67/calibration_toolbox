"""
Get the transformation from marker to the camera.
@author: Hongtao Wu
Nov 15, 2019
"""

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from ar_track_alvar_msgs import AlvarMarkers

from threading import Lock
import numpy as np
import time
import cv2

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class Alvar:
    def __init__(self):
        self.pose_topic = "/ar_pose_marker"
    
        self._pose_sub = rospy.Subscriber(self.pose_topic, AlvarMarkers, self._poseInfoCb)

        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.pose_qw = None
        self.pose_qx = None
        self.pose_qy = None
        self.pose_qz = None
        
        self.pos = None
        self.orn = None

        self.mutex = Lock()


    def _poseInfoCb(self, msg):
        if msg is None:
            rospy.logwarn("_poseInfoCb: msg is None!")
        
        with self.mutex:        
            self.pose_x = msg.markers[0].pose.position.x
            self.pose_y = msg.markers[0]pose.position.y
            self.pose_z = msg.markers[0]pose.position.z
            self.pose_qx = msg.markers[0].pose.orientation.x
            self.pose_qy = msg.markers[0].pose.orientation.y
            self.pose_qz = msg.markers[0].pose.orientation.z
            self.pose_qw = msg.markers[0].pose.orientation.w
            self.pos = np.array([self.pose_x, self.pose_y, self.pose_z])
            self.orn = np.array([self.pose_qw, self.pose_qx, self.pose_qy, self.pose_qz])

    def get_pose(self):
        pos = None
        orn = None

        with self.mutex:
            if (self.pos is not None) and (self.orn is not None):
                pos = self.pos.copy()
                orn = self.orn.copy()

        return pos, orn
