#! /usr/bin/python

"""
Collect data for estimating intrinsic of the RGB and IR camera

Written by Hongtao Wu, Johns Hopkins University
Oct 04, 2020
"""

import os
import time
import cv2
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from calibration_toolbox.srv import *

class DataCollectServer(object):
    
    def __init__(self):
        self.rgb_image_raw_topic = "/camera/color/image_raw"
        self.ir_image_raw_topic  = "/camera/ir/image"
        
        # Primesense cannot stream IR and RGB at the same time.
        # The subscribers are established when they need to
        self.ir_image_raw_sub = None
        self.rgb_image_raw_sub = None
        
        self.cv_rgb_img = None
        self.cv_ir_img  = None

        self.bridge = CvBridge()

    def rgb_image_raw_cb(self, msg):
        if msg is None:
            rospy.logwarn("No rgb images recevied!!!")
        
        try:
            self.cv_rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def ir_image_raw_cb(self, msg):
        if msg is None:
            rospy.logwarn("No ir images received!!!")

        try:
            self.cv_ir_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except CvBridgeError as e:
            rospy.logwarn(str(e))
    
    def handle_data_collector(self, req):
        """
        Save the rgb and ir images into req.data_dir
        """
        self.ir_image_raw_sub  = rospy.Subscriber(self.ir_image_raw_topic, Image, self.ir_image_raw_cb)
        time.sleep(1)
        self.ir_image_raw_sub.unregister()

        ir_path = os.path.join(req.data_dir, 'ir' + str(req.prefix) + '.jpg')
        cv2.imwrite(ir_path, self.cv_ir_img)

        rospy.loginfo("Finish collecting {}-th IR image".format(req.prefix))
        # time.sleep(0.5)

        # self.rgb_image_raw_sub = rospy.Subscriber(self.rgb_image_raw_topic, Image, self.rgb_image_raw_cb)
        # time.sleep(1)
        # self.rgb_image_raw_sub.unregister()

        # rgb_path = os.path.join(req.data_dir, 'rgb' + str(req.prefix) + '.jpg')
        # cv2.imwrite(rgb_path, self.cv_rgb_img)

        # rospy.loginfo("Finish collecting {}-th RGB image".format(req.prefix))

    def run_data_collector_server(self):
        s = rospy.Service("collect_data", DataCollect, self.handle_data_collector)

        rospy.loginfo ("Ready to calibration!")
        rospy.loginfo ("Please move the robot to different config to collect images for Intrinsic Calibration")

        rospy.spin()
  

# (1.261516809463501, -2.168732468281881, 2.6401619911193848, -3.3979209105121058, -1.2012608687030237, -0.8512676397906702)