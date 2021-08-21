# Calibration collector for chessboard and ArUco Tag
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
from calibration_toolbox.alvar import Alvar
from calibration_toolbox.ros_camera import ROSCamera

from calibration_toolbox.srv import *

class CalibrateCollector(object):
    
    def __init__(self, target, calib_points_file, image_topic, base_frame, ee_frame, camera_frame):
        """If the target is chessboard, only images and robot poses will be collected.
        The pose of the chess board will be detected later using the chessboard_detector.py
        If the target is aruco, images, marker poses, and robot poses will be collected.

        Args:
            target (string): chessboard / aruco
            calib_points_file (string): files of points for calibration
            image_topic (string): ros topic for the image source
            base_frame (string): robot base frame
            ee_frame (string): end effector frame
        """
        rospy.loginfo("Make sure to roslaunch realsense2_camera rs_camera.launch before running this code!")
        rospy.loginfo("Make sure to roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>")

        self.target = target
        self.image_topic = image_topic

        # Load calib_points
        self.calib_points = np.loadtxt(calib_points_file, delimiter=',')
        self.calib_points_num = self.calib_points.shape[0]

        # Specify the names of the frames in TF
        self.base_frame_name = base_frame
        self.ee_frame_name = ee_frame
        self.camera_frame_name = camera_frame

        if self.target == 'aruco':
            rospy.loginfo("Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>")
            self.markerIncam_pos = None
            self.markerIncam_orn = None
            self.markerIncam_mat = None
            self.ar_marker_frame_name = "aruco_marker_frame"
        
        if self.target == 'alvar':
            rospy.loginfo("Make sure to roslaunch ar_track_alvar pr2_indiv_no_kinect.launch")
            self.markerIncam_pos = None
            self.markerIncam_orn = None
            self.markerIncam_mat = None
            self.ar_marker_frame_name = "alvar_marker_frame"

        # Initialize TF
        self.tfBuffer = tf2_ros.Buffer()
        self.robot_pose_listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize the robot
        self.robot = PandaRobot(base_frame=self.base_frame_name,
                                ee_frame=self.ee_frame_name,
                                camera_frame=self.camera_frame_name)
                                
        self.camera_handler = None

        self.robot_poses = []
        self.marker_poses = []

        self.cam2ee = None

        self.data_dir = None

    def get_marker_2_cam(self):
        """Get the transformation of the marker in camera frame."""

        time.sleep(0.5)

        markerIncam_pos, markerIncam_quat, aruco_img = self.camera_handler.get_pose()

        # If the difference between the previous marker pos and the current is large
        # Consider it got a new detection of tag
        if markerIncam_pos is not None:
            self.markerIncam_pos = markerIncam_pos
            self.markerIncam_orn = quat2rotm(markerIncam_quat)
            self.markerIncam_mat = make_rigid_transformation(self.markerIncam_pos, self.markerIncam_orn)
        else:
            self.markerIncam_mat = None

        return self.markerIncam_mat, aruco_img

    def save_transforms_to_file_marker(self, save_dir, calib_pt_idx, tool_transformation, marker_transformation, marker_img):
        """Save the transformation to files."""

        robot_pose_file = os.path.join(save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        marker_pose_file = os.path.join(save_dir, str(calib_pt_idx) + '_markerpose.txt')
        marker_img_file = os.path.join(save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        # Marker pose in camera frame
        with open(marker_pose_file, 'w') as file2:
            for l in np.reshape(marker_transformation, (16, )).tolist():
                file2.writelines(str(l) + ' ')
        
        if marker_img is not None:
            cv2.imwrite(marker_img_file, marker_img)

    def save_transforms_to_file_chessboard(self, save_dir, calib_pt_idx, tool_transformation, chessboard_img):
        """Save the transformation to files."""

        robot_pose_file = os.path.join(save_dir, str(calib_pt_idx)  + '_robotpose.txt')
        chessboard_img_file = os.path.join(save_dir, str(calib_pt_idx) + '_img.png')
        
        # Tool pose in robot base frame
        with open(robot_pose_file, 'w') as file1:
            for l in np.reshape(tool_transformation, (16, )).tolist():
                file1.writelines(str(l) + ' ')
        
        if chessboard_img is not None:
            cv2.imwrite(chessboard_img_file, chessboard_img)

    def collect_data(self, req):
        """Collect data for calibration."""        

        # Initialize camera handler
        if self.target == 'aruco':
            self.camera_handler = ArUco()
        elif self.target == 'alvar':
            self.camera_handler = Alvar(img_topic=self.image_topic)
        else:
            self.camera_handler = ROSCamera(img_topic=self.image_topic)

        time.sleep(0.5)
    
        complete_point_num = 0
        rospy.loginfo(self.calib_points_num)
        
        while complete_point_num < self.calib_points_num:
            print ("Pose index: ", complete_point_num)

            # Move to joint config
            joint_config = self.calib_points[complete_point_num, :]
            self.robot.moveToJointPosition(joint_config)

            time.sleep(3)

            if self.target == 'aruco' or self.target == 'alvar':
                # Marker Pose and Image
                marker_pose, marker_img = self.get_marker_2_cam()

                if marker_pose is not None:
                    # Robot Pose
                    transform_ros = self.tfBuffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rospy.Time(0))

                    pos_ros = transform_ros.transform.translation
                    quat_ros = transform_ros.transform.rotation

                    robot_pos = np.array([pos_ros.x, pos_ros.y, pos_ros.z])
                    robot_quat = np.array([quat_ros.w, quat_ros.x, quat_ros.y, quat_ros.z])
                    robot_rotm = quat2rotm(robot_quat)
                    robot_pose = make_rigid_transformation(robot_pos, robot_rotm)

                    self.save_transforms_to_file_marker(req.data_path, complete_point_num, robot_pose, marker_pose, marker_img)
                    complete_point_num += 1

                    print ("===============================")
                else:
                    print ("Marker pose is None! The camera probably cannot see it!")
            

            else:
                # Image
                chessboard_img = self.camera_handler.get_img()

                time.sleep(0.5)
                
                if chessboard_img is not None:
                    # Robot Pose
                    transform_ros = self.tfBuffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rospy.Time(0))

                    pos_ros = transform_ros.transform.translation
                    quat_ros = transform_ros.transform.rotation

                    robot_pos = np.array([pos_ros.x, pos_ros.y, pos_ros.z])
                    robot_quat = np.array([quat_ros.w, quat_ros.x, quat_ros.y, quat_ros.z])
                    robot_rotm = quat2rotm(robot_quat)
                    robot_pose = make_rigid_transformation(robot_pos, robot_rotm)

                    self.save_transforms_to_file_chessboard(req.data_path, complete_point_num, robot_pose, chessboard_img)
                    complete_point_num += 1

                    print ("===============================")
            
            time.sleep(1)

        return CollectDataResponse("Successfully collect data! Proceed to calibrate data.")

    def run(self):
        """Run the server."""

        s = rospy.Service("collect_data", CollectData, self.collect_data)
        rospy.loginfo("Ready to collect data for calibration...")

        rate = rospy.Rate(10)
        rospy.spin()