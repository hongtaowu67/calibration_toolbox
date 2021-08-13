# Class to interact with a camera in ROS
# Author: Hongtao Wu
# Johns Hopkins University
# National University of Singapore
# Date: Aug 13, 2021

import rospy
import roslib
from sensor_msgs.msg import Image

from threading import Lock
import numpy as np
import cv2

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


class ROSCamera(object):

    def __init__(self, img_topic='/camera/color/image_raw'):
        """Class to interact with a cameara in ROS

        Args:
            img_topic (string): ros topic of the image you want to subscribe to
        """
        self.img_topic = img_topic
    
        self._img_sub = rospy.Subscriber(self.img_topic, Image, self._imgCb)

        self.img = None

        self.mutex = Lock()
        self._bridge = CvBridge()

    def _imgCb(self, msg):
        """Image callback function."""

        if msg is None:
            rospy.logwarn("_arucoimgCb: msg is None !!!!!!!!!")
        try:
            # max out at 10 hz assuming 30hz data source
            if msg.header.seq % 3 == 0:
                cv_image = self._bridge.imgmsg_to_cv2(msg, "rgb8")

                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                with self.mutex:
                    self.img = cv_image

        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def get_img(self):
        """Get the img of the current frame."""

        img = None

        with self.mutex:
            if self.img is not None:
                img = self.img

        return img