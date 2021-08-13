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
    
    def __init__(self, calib_point_num=31):
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
        # self.calib_joint_configs = []
        # self.calib_joint_configs.append([0.829402,-0.830306,-0.955126,-2.30334,0.202149,2.84722,0.371086])
        # self.calib_joint_configs.append([0.708371,-0.625007,-0.837845,-2.2098,0.204379,2.75336,0.384986])
        # self.calib_joint_configs.append([0.331349,-0.419167,-0.502453,-2.20837,0.288984,2.83689,0.425098])
        # self.calib_joint_configs.append([0.00384433,-0.381613,-0.204643,-2.22471,0.288779,2.84424,0.414743])
        # self.calib_joint_configs.append([-0.397768,-0.412427,0.165338,-2.22101,0.337444,2.76917,0.430895])
        # self.calib_joint_configs.append([-0.712382,-0.486738,0.450862,-2.2542,0.38736,2.76911,0.425198])
        # self.calib_joint_configs.append([-1.05852,-0.507805,0.756403,-2.17156,0.492114,2.76529,0.410753])
        # self.calib_joint_configs.append([-1.04658,-0.584379,0.822992,-2.34864,0.647245,3.09791,0.426993])
        # self.calib_joint_configs.append([-0.739075,-0.523928,0.583049,-2.41125,0.640788,3.15315,0.386228])
        # self.calib_joint_configs.append([-0.249173,-0.513014,0.133767,-2.48405,0.640839,3.17061,0.312543])
        # self.calib_joint_configs.append([0.463652,-0.604758,-0.540549,-2.47571,0.502429,3.23987,0.258027])
        # self.calib_joint_configs.append([0.798493,-0.780569,-0.858468,-2.44979,0.444323,3.24063,0.190954])
        # self.calib_joint_configs.append([1.00837,-0.972864,-1.01867,-2.38373,0.467672,3.15598,0.38312])
        # self.calib_joint_configs.append([1.10153,-0.989677,-1.06994,-2.3485,0.467837,3.20543,0.889421])
        # self.calib_joint_configs.append([1.2227,-1.33822,-1.14429,-2.35385,0.422949,3.11821,1.48888])
        # self.calib_joint_configs.append([1.36922,-1.58185,-1.07577,-2.55801,0.403564,3.68637,1.87198])
        # self.calib_joint_configs.append([1.08673,-1.1105,-1.13312,-2.25823,-2.71144,2.82304,2.75649])
        # self.calib_joint_configs.append([0.973297,-1.06328,-1.03306,-2.37245,-2.48352,2.6808,2.66558])
        # self.calib_joint_configs.append([1.19078,-1.47857,-1.01678,-2.54689,-2.50715,2.19507,2.30742])
        # self.calib_joint_configs.append([1.40047,-1.72799,-1.03042,-2.63971,-2.72201,1.81759,2.05533])
        # self.calib_joint_configs.append([1.49983,-1.72976,-1.06856,-2.62111,-2.86228,1.68168,2.03683])
        # self.calib_joint_configs.append([1.56286,-1.75434,-1.04047,-2.61007,-2.83381,1.61209,2.0615])
        # self.calib_joint_configs.append([1.25359,-1.65926,-1.49284,-2.42242,-1.33679,3.52806,0.902418])
        # self.calib_joint_configs.append([1.48087,-1.71305,-1.44124,-2.42384,-0.817547,3.70196,1.47299])
        # self.calib_joint_configs.append([0.683883,-0.680211,-0.693401,-2.4725,-2.50852,2.31934,2.43279])
        # self.calib_joint_configs.append([-0.406396,-0.656666,0.329129,-2.62292,-2.46446,2.35652,2.48868])
        # self.calib_joint_configs.append([-1.08301,-0.955662,0.955488,-2.644,-2.46972,2.35697,2.50807])
        # self.calib_joint_configs.append([-1.32729,-0.968222,1.11532,-2.50242,-2.50447,2.37501,2.60316])
        # self.calib_joint_configs.append([-1.51469,-1.30904,1.25157,-2.50609,-2.5042,2.37567,2.66389])
        # self.calib_joint_configs.append([-1.61974,-1.69242,1.33846,-2.58355,-2.50497,2.37295,2.58779])
        # self.calib_joint_configs.append([-1.74043,-1.63069,1.27608,-2.5074,-2.82839,2.22068,2.73851])

        self.calib_point_num = len(self.calib_joint_configs)

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
        rospy.loginfo(self.calib_point_num)
        
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

        # self.robot.goHome()
        return CollectDataResponse("Successfully collect data! Proceed to calibrate data.")

    def run(self):
        """Run the server."""

        s = rospy.Service("collect_data", CollectData, self.collect_data)
        rospy.loginfo("Ready to collect data for ArUco...")

        rate = rospy.Rate(10)
        rospy.spin()