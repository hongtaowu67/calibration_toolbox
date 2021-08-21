# Class to interact with the ArUco tag
# Author: Hongtao Wu
# Johns Hopkins University
# Date: Nov 15, 2019

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image

from threading import Lock
import numpy as np
import time
import cv2
import os

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class Alvar(object):
    """Class to interact with Alvar tag."""

    def __init__(self, img_topic):
        self.pose_topic = "/ar_pose_marker"
        self._img_topic = img_topic
    
        self._img_sub = rospy.Subscriber(self._img_topic, Image, self._imgCb)
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

        self._img = None

        self.mutex = Lock()
        self._bridge = CvBridge()

    def _poseInfoCb(self, msg):
        """Pose callback function."""

        if msg is None:
            rospy.logwarn("_poseInfoCb: msg is None!")
        
        if len(msg.markers) < 1:
            self.pos = None
            self.orn = None
        else:     
            with self.mutex:        
                self.pose_x = msg.markers[0].pose.pose.position.x
                self.pose_y = msg.markers[0].pose.pose.position.y
                self.pose_z = msg.markers[0].pose.pose.position.z
                self.pose_qx = msg.markers[0].pose.pose.orientation.x
                self.pose_qy = msg.markers[0].pose.pose.orientation.y
                self.pose_qz = msg.markers[0].pose.pose.orientation.z
                self.pose_qw = msg.markers[0].pose.pose.orientation.w
                self.pos = np.array([self.pose_x, self.pose_y, self.pose_z])
                self.orn = np.array([self.pose_qw, self.pose_qx, self.pose_qy, self.pose_qz])

    def _imgCb(self, msg):
        """Image callback function."""

        if msg is None:
            rospy.logwarn("_imgCb: msg is None !!!!!!!!!")
        try:
            # max out at 10 hz assuming 30hz data source
            if msg.header.seq % 3 == 0:
                cv_image = self._bridge.imgmsg_to_cv2(msg, "rgb8")
                # decode the data, this will take some time

                # rospy.loginfo('rgb color cv_image shape: ' + str(cv_image.shape) + ' depth sequence number: ' + str(msg.header.seq))
                # print('rgb color cv_image shape: ' + str(cv_image.shape))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                # rgb_img = cv2.imencode('.jpg', cv_image)[1].tobytes()
                # rgb_img = GetJpeg(np.asarray(cv_image))

                with self.mutex:
                    self._img = cv_image
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def get_pose(self):
        """Get the pose of the current frame."""

        pos = None
        orn = None
        _img = None

        with self.mutex:
            if (self.pos is not None) and (self.orn is not None):
                pos = self.pos.copy()
                orn = self.orn.copy()
                _img = self._img

        return pos, orn, _img
