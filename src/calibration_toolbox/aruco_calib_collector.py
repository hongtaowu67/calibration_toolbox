# Calibration with ArUco tag
# Author: Hongtao Wu, Xin Meng
# Johns Hopkins University
# National University of Singapore
# Date: July 15, 2021

import time
import numpy as np
import cv2
import os
import yaml

import rospy
import tf2_ros
import tf.transformations as transformations
import geometry_msgs.msg

from calibration_toolbox.utils import *
from calibration_toolbox.panda_robot import PandaRobot
from calibration_toolbox.aruco import ArUco

from calibration_toolbox.srv import *

class ArucoCalibrateCollector(object):
    
    def __init__(self, calib_point_num=15):
        rospy.loginfo("Make sure to roslaunch realsense2_camera rs_camera.launch before running this code!")
        rospy.loginfo("Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>")
        rospy.loginfo("Make sure to roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>")
        
        self.robot = None
        self.aruco = None

        self.markerIncam_pos = None
        self.markerIncam_orn = None
        self.markerIncam_mat = None

        self.calib_point_num = calib_point_num

        self.base_frame_name = "panda_link0"
        self.ee_frame_name = "panda_EE"
        self.camera_frame_name = "camera_color_optical_frame"
        self.ar_marker_frame_name = "aruco_marker_frame"

        self.tfBuffer = tf2_ros.Buffer()
        self.robot_pose_listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize the robot
        self.robot = PandaRobot()

        # 15 points for calibration
        # Values are obtained from moving the robot in teaching mode
        # and read the value with ./path/to/libfranka/build/examples/echo_robot_state <robot_ip>

        # panda with RS
        self.calib_joint_configs = []
        self.calib_joint_configs.append([-0.146975,0.635671,0.184451,-1.38386,0.0205211,3.00829,0.265557])
        self.calib_joint_configs.append([-0.171785,0.634595,0.183138,-1.38399,0.0274045,2.99998,0.507089])
        self.calib_joint_configs.append([-0.236165,0.625268,0.263404,-1.41085,0.248626,2.99773,0.510317])
        self.calib_joint_configs.append([-0.262306,0.625756,0.305554,-1.41413,0.424335,2.99442,0.520265])
        self.calib_joint_configs.append([-0.488863,0.814713,0.694643,-1.36112,0.303203,3.17385,1.45979])
        self.calib_joint_configs.append([-0.517351,0.795696,0.734402,-1.51093,0.315584,3.45465,1.46796])
        self.calib_joint_configs.append([-0.435564,0.757529,0.66653,-1.59285,0.301739,3.64884,1.566])
        self.calib_joint_configs.append([0.329533,0.708727,-0.24378,-1.50073,0.799596,3.59342,1.79216])
        self.calib_joint_configs.append([0.630784,0.765141,-0.655252,-1.60885,0.817361,3.59371,1.91854])
        self.calib_joint_configs.append([0.53397,1.08556,-0.728472,-1.229,0.81158,3.59576,1.80689])
        self.calib_joint_configs.append([-0.030525,0.931611,0.157928,-1.12151,0.531689,3.46992,1.68935])
        self.calib_joint_configs.append([-0.471302,1.02723,0.797877,-1.23677,0.00838211,3.46727,1.70088])
        self.calib_joint_configs.append([-0.759316,1.00404,1.01275,-1.54098,-0.215028,3.46683,1.70687])
        self.calib_joint_configs.append([-0.826864,1.23562,1.24963,-1.49232,-0.221334,3.46774,1.7015])
        self.calib_joint_configs.append([-0.538093,1.38998,1.23818,-1.03671,-0.237198,3.48952,1.54483])


        self.robot_poses = []
        self.marker_poses = []

        self.cam2ee = None

        self.data_dir = None

    def get_marker_2_cam(self):
        """Get the transformation of the marker in camera frame."""

        time.sleep(0.5)

        markerIncam_pos, markerIncam_quat, aruco_img = self.aruco.get_pose()

        # If the difference between the previous marker pos and the current is large
        # Consider it got a new detection of tag
        if markerIncam_pos is not None:
            self.markerIncam_pos = markerIncam_pos
            self.markerIncam_orn = quat2rotm(markerIncam_quat)
            self.markerIncam_mat = make_rigid_transformation(self.markerIncam_pos, self.markerIncam_orn)
        else:
            self.markerIncam_mat = None

        return self.markerIncam_mat, aruco_img

    def save_transforms_to_file(self, save_dir, calib_pt_idx, tool_transformation, marker_transformation, aruco_img):
        """Save the transformation to files."""

        robot_pose_file = os.path.join(save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        marker_pose_file = os.path.join(save_dir, str(calib_pt_idx) + '_markerpose.txt')
        aruco_img_file = os.path.join(save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        # Marker pose in camera frame
        with open(marker_pose_file, 'w') as file2:
            for l in np.reshape(marker_transformation, (16, )).tolist():
                file2.writelines(str(l) + ' ')
        
        if aruco_img is not None:
            cv2.imwrite(aruco_img_file, aruco_img)

    def collect_data(self, req):
        """Collect data for calibration."""        

        # Initialize the aruco
        self.aruco = ArUco()
        time.sleep(0.5)
        
        epsilon = 0.05 # randomness for sampling orientation
        complete_point_num = 0
        
        while complete_point_num < self.calib_point_num:
            print ("Pose index: ", complete_point_num)

            # Move to joint config
            joint_config = self.calib_joint_configs[complete_point_num]
            self.robot.moveToJointPosition(joint_config)

            time.sleep(3)

            # Marker Pose
            marker_pose, aruco_img = self.get_marker_2_cam()

            if marker_pose is not None:
                # Robot Pose
                transform_ros = self.tfBuffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rospy.Time(0))
                # print ("Ros transform: ", transform_ros)

                pos_ros = transform_ros.transform.translation
                quat_ros = transform_ros.transform.rotation

                robot_pos = np.array([pos_ros.x, pos_ros.y, pos_ros.z])
                # print ("Ros robot position")
                # print (robot_pos)
                robot_quat = np.array([quat_ros.w, quat_ros.x, quat_ros.y, quat_ros.z])
                robot_rotm = quat2rotm(robot_quat)
                # print ("Ros robot rotation matrix")
                # print (robot_rotm)
                robot_pose = make_rigid_transformation(robot_pos, robot_rotm)

                print ("===============================")

                self.save_transforms_to_file(req.data_path, complete_point_num, robot_pose, marker_pose, aruco_img)

                complete_point_num += 1
            else:
                print ("Marker pose is None! The camera probably cannot see it!")
            
            time.sleep(1)

        self.robot.goHome()
        return CollectDataResponse("Successfully collect data! Proceed to calibrate data.")

    def run(self):
        """Run the server."""

        s = rospy.Service("collect_data", CollectData, self.collect_data)
        rospy.loginfo("Ready to collect data for ArUco...")

        rate = rospy.Rate(10)
        rospy.spin()