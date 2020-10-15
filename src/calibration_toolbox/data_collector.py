#! /usr/bin/python

"""
Collect data for estimating intrinsic of the RGB and IR camera

Written by Hongtao Wu, Johns Hopkins University
Oct 04, 2020
"""

import os
import time
import cv2
import pcl

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from calibration_toolbox.srv import *

class DataCollectServer(object):
    
    def __init__(self):
        # self.rgb_image_topic = "/camera/color/image_raw"
        # self.ir_image_raw_topic  = "/camera/aligned_depth_to_color/image_raw"
        # self.pc_topic = "/camera/depth_registered/points"

        self.rgb_image_topic = "/camera/rgb/image_rect_color"
        self.depth_reg_image_topic = "/camera/depth_registered/sw_registered/image_rect"
        self.depth_image_topic = "/camera/depth/image_rect_raw"
        self.pc_reg_topic = "/camera/depth_registered/points"
        self.pc_topic = "/camera/depth/points"
        
        # Primesense cannot stream IR and RGB at the same time.
        # The subscribers are established when they need to
        self.depth_reg_image_sub = None
        self.depth_image_sub = None
        self.rgb_image_sub = None
        self.pc_sub = None
        self.pc_reg_sub = None
        
        self.cv_rgb_img = None
        self.cv_depth_reg_img  = None
        self.cv_depth_img = None
        self.pc = None
        self.pc_reg = None

        self.bridge = CvBridge()

    def rgb_image_cb(self, msg):
        if msg is None:
            rospy.logwarn("No rgb images recevied!!!")
        
        try:
            self.cv_rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def depth_reg_image_cb(self, msg):
        if msg is None:
            rospy.logwarn("No ir images received!!!")

        try:
            rospy.loginfo("depth registered frame id: {}".format(msg.header.frame_id))
            self.cv_depth_reg_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def depth_image_cb(self, msg):
        if msg is None:
            rospy.logwarn("No ir images received!!!")

        try:
            rospy.loginfo("depth frame id: {}".format(msg.header.frame_id))
            self.cv_depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def pc_cb(self, msg):
        if msg is None:
            rospy.logwarn("No pc received!!!")
        else:
            rospy.loginfo("get pc!")
            rospy.loginfo("Frame ID: {}".format(msg.header.frame_id))
            self.pc = self.ros_to_pcl(msg)
    
    def pc_reg_cb(self, msg):
        if msg is None:
            rospy.logwarn("No pc received!!!")
        else:
            rospy.loginfo("get pc reg!")
            rospy.loginfo("Frame ID: {}".format(msg.header.frame_id))
            self.pc_reg = self.ros_to_pcl_rgb(msg)
        
    def ros_to_pcl_rgb(self, ros_pc):
        points_list = []
        for data in pc2.read_points(ros_pc, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        return pcl_data

    def ros_to_pcl(self, ros_pc):
        points_list = []
        for data in pc2.read_points(ros_pc, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        return pcl_data
    
    def handle_data_collector(self, req):
        """
        Save the rgb and ir images into req.data_dir
        """
        ####
        self.depth_reg_image_sub  = rospy.Subscriber(self.depth_reg_image_topic, Image, self.depth_reg_image_cb)
        time.sleep(1)
        self.depth_reg_image_sub.unregister()

        depth_reg_path = os.path.join(req.data_dir, 'depth_registered_' + str(req.prefix) + '.png')
        cv2.imwrite(depth_reg_path, self.cv_depth_reg_img)

        rospy.loginfo("Finish collecting {}-th depth registered image".format(req.prefix))
        time.sleep(0.5)

        ####
        self.depth_image_sub  = rospy.Subscriber(self.depth_image_topic, Image, self.depth_image_cb)
        time.sleep(1)
        self.depth_image_sub.unregister()

        depth_path = os.path.join(req.data_dir, 'depth_' + str(req.prefix) + '.png')
        cv2.imwrite(depth_path, self.cv_depth_img)

        rospy.loginfo("Finish collecting {}-th depth image".format(req.prefix))
        time.sleep(0.5)

        ####
        self.rgb_image_sub = rospy.Subscriber(self.rgb_image_topic, Image, self.rgb_image_cb)
        time.sleep(1)
        self.rgb_image_sub.unregister()

        rgb_path = os.path.join(req.data_dir, 'rgb_' + str(req.prefix) + '.png')
        cv2.imwrite(rgb_path, self.cv_rgb_img)

        rospy.loginfo("Finish collecting {}-th color image".format(req.prefix))

        ####
        self.pc_sub = rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_cb)
        time.sleep(2)

        pc_path = os.path.join(req.data_dir, 'pc_' + str(req.prefix) + '.ply')
        pcl.save(self.pc, pc_path, format='ply')

        self.pc_sub.unregister()
        rospy.loginfo("Finish collecting {}-th point cloud".format(req.prefix))

        ####
        self.reg_pc_sub = rospy.Subscriber(self.pc_reg_topic, PointCloud2, self.pc_reg_cb)
        time.sleep(3)

        pc_reg_path = os.path.join(req.data_dir, 'pc_reg_' + str(req.prefix) + '.ply')
        pcl.save(self.pc_reg, pc_reg_path, format='ply')
        
        self.reg_pc_sub.unregister()
        rospy.loginfo("Finish collecting {}-tha registered point cloud".format(req.prefix))

    def run_data_collector_server(self):
        s = rospy.Service("collect_data", DataCollect, self.handle_data_collector)

        rospy.loginfo ("Ready to calibration!")
        rospy.loginfo ("Please move the robot to different config to collect images for Intrinsic Calibration")

        rospy.spin()
  