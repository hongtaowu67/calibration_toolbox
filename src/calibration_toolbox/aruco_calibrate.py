from __future__ import print_function, division

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

class Calibrate(object):
    
    def __init__(self, calib_point_num=15):
        print ("Make sure to roslaunch realsense2_camera rs_camera.launch before running this code!")
        print ("Make sure to roslaunch aruco_ros single.launch markerId:=<markerId> markerSize:=<markerSize>")
        print ("Make sure to roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>")
        
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
        """Collect data for calibration
        """        
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


    def collect_data_alvar(self, req):
        """
        Collect data for calibration
        """        
        time.sleep(0.5)
        complete_point_num = 0
        
        while complete_point_num < self.calib_point_num:
            print ("Pose index: ", complete_point_num)

            # Move to joint config
            joint_config = self.calib_joint_configs[complete_point_num]
            self.robot.moveToJointPosition(joint_config)

            time.sleep(3)

            # Marker Pose
            marker_tf = self.tfBuffer.lookup_transform(self.camera_frame_name, self.ar_marker_frame_name, rospy.Time(0))

            # Robot Pose
            robot_tf = self.tfBuffer.lookup_transform(self.base_frame_name, self.ee_frame_name, rospy.Time(0))

            marker_pos_ros  = marker_tf.transform.translation
            marker_quat_ros = marker_tf.transform.rotation
            robot_pos_ros   = robot_tf.transform.translation
            robot_quat_ros  = robot_tf.transform.rotation

            marker_pos = np.array([marker_pos_ros.x, marker_pos_ros.y, marker_pos_ros.z])
            robot_pos  = np.array([robot_pos_ros.x, robot_pos_ros.y, robot_pos_ros.z])
            
            print ("ROS marker position")
            print (marker_pos)
            print ("ROS robot position")
            print (robot_pos)

            marker_quat = np.array([marker_quat_ros.w, marker_quat_ros.x, marker_quat_ros.y, marker_quat_ros.z])
            marker_rotm = quat2rotm(marker_quat)
            robot_quat  = np.array([robot_quat_ros.w, robot_quat_ros.x, robot_quat_ros.y, robot_quat_ros.z])
            robot_rotm  = quat2rotm(robot_quat)

            marker_pose = make_rigid_transformation(marker_pos, marker_rotm)
            robot_pose  = make_rigid_transformation(robot_pos, robot_rotm)

            print ("===============================")

            self.save_transforms_to_file(req.data_path, complete_point_num, robot_pose, marker_pose, aruco_img=None)

            complete_point_num += 1
            
            time.sleep(1)

        self.robot.goHome()
        return CollectDataResponse("Successfully collect data! Proceed to calibrate data.")


    def load_transforms(self, load_dir):
        """ Load robot pose and marker pose from a save directory
        # Arguments
        load_dir: the directory where calibration data was previously acquired from the robot.
        # Returns
        Two lists of 4x4 transformation matrices.
        self.robot_poses, self.marker_poses
        """

        for f in os.listdir(load_dir):
            if 'robotpose.txt' in f:
                robot_pose_file = f
                marker_pose_file = f[:-13] + 'markerpose.txt'

                # tool pose in robot base frame
                with open(os.path.join(load_dir, robot_pose_file), 'r') as file_robot:
                    robotpose_str = file_robot.readline().split(' ')
                    robotpose = [float (x) for x in robotpose_str if x is not '']
                    assert len(robotpose) == 16
                    robotpose = np.reshape(np.array(robotpose), (4, 4))
                self.robot_poses.append(robotpose)

                # marker pose in camera frame
                with open(os.path.join(load_dir, marker_pose_file), 'r') as file_marker:
                    markerpose_str = file_marker.readline().split(' ')
                    markerpose = [float(x) for x in markerpose_str if x is not '']
                    assert len(markerpose) == 16
                    markerpose = np.reshape(np.array(markerpose), (4, 4))
                self.marker_poses.append(markerpose)
    
    def axxb(self):
        """
        AX=XB solver.
        
        Args:
        - self.robot_poses (list of 4x4 numpy array): poses (homogenous transformation) of the robot end-effector in the robot base frame.
        - self.marker_poses (list of 4x4 numpy array): poses (homogenous transformation) of the marker in the camera frame.
        Return:
        - self.cam2ee (4x4 numpy array): poses of the camera in the robot end-effector frame.
        """

        assert len(self.robot_poses) == len(self.marker_poses), 'robot poses and marker poses are not of the same length!'

        n = len(self.robot_poses)
        print("Pose Pair Num: {}".format(n))

        pose_inds= np.arange(n)
        np.random.shuffle(pose_inds)

        print ("Total Pose: %i" % n)
        A = np.zeros((4, 4, n-1))
        B = np.zeros((4, 4, n-1))
        alpha = np.zeros((3, n-1))
        beta = np.zeros((3, n-1))

        M = np.zeros((3, 3))

        for i in range(n-1):
            A[:, :, i] = np.matmul(pose_inv(self.robot_poses[pose_inds[i+1]]), self.robot_poses[pose_inds[i]])
            B[:, :, i] = np.matmul(self.marker_poses[pose_inds[i+1]], pose_inv(self.marker_poses[pose_inds[i]]))
            alpha[:, i] = get_mat_log(A[:3, :3, i])
            beta[:, i] = get_mat_log(B[:3, :3, i])
            M += np.outer(beta[:, i], alpha[:, i])

            # Bad pair of transformation are very close in the orientation.
            # They will give nan result
            if np.sum(np.isnan(alpha[:, i])) + np.sum(np.isnan(beta[:, i])) > 0:
                nan_num += 1
                continue
            else:
                M += np.outer(beta[:, i], alpha[:, i])

        # Get the rotation matrix
        mtm = np.matmul(M.T, M)
        u_mtm, s_mtm, vh_mtm = np.linalg.svd(mtm)
        R = np.matmul(np.matmul(np.matmul(u_mtm, np.diag(np.power(s_mtm, -0.5))), vh_mtm), M.T)

        # Get the tranlation vector
        I_Ra_Left = np.zeros((3*(n-1), 3))
        ta_Rtb_Right = np.zeros((3 * (n-1), 1))
        for i in range(n-1):
            I_Ra_Left[(3*i):(3*(i+1)), :] = np.eye(3) - A[:3, :3, i]
            ta_Rtb_Right[(3*i):(3*(i+1)), :] = np.reshape(A[:3, 3, i] - np.dot(R, B[:3, 3, i]), (3, 1))
        

        t = np.linalg.lstsq(I_Ra_Left, ta_Rtb_Right, rcond=-1)[0]
        
        self.cam2ee = np.c_[R, t]
        self.cam2ee = np.r_[self.cam2ee, [[0, 0, 0, 1]]]

        print ("Calibration Result:\n", self.cam2ee)

    def test(self):
        """ Test the accuracy of the calculated result.
            Use AXB to calculate the base2tag transformation for each frame.
        """
        num_poses = 0

        for f in os.listdir(self.data_dir):
            if 'robotpose.txt' in f:
                robot_pose_file = f
                marker_pose_file = f[:-13] + 'markerpose.txt'

                # tool pose in robot base frame
                with open(os.path.join(self.data_dir, robot_pose_file), 'r') as file_robot:
                    robotpose_str = file_robot.readline().split(' ')
                    robotpose = [float (x) for x in robotpose_str if x is not '']
                    assert len(robotpose) == 16
                    robotpose = np.reshape(np.array(robotpose), (4, 4))

                # marker pose in camera frame
                with open(os.path.join(self.data_dir, marker_pose_file), 'r') as file_marker:
                    markerpose_str = file_marker.readline().split(' ')
                    markerpose = [float(x) for x in markerpose_str if x is not '']
                    assert len(markerpose) == 16
                    markerpose = np.reshape(np.array(markerpose), (4, 4))

                base2tag = np.matmul(np.matmul(robotpose, self.cam2ee), markerpose)
                
                base2tag_rotm = base2tag[:3, :3]
                base2tag_aa   = rotm2angle(base2tag_rotm)
                base2tag_pos  = base2tag[:3, -1] 
                
                print(f)
                # print ("xform")
                # print(base2tag)
                print("aa: {}".format(base2tag_aa))
                print("pos: {}".format(base2tag_pos))

                num_poses += 1
        
        print("Total Pose Pair Num: {}".format(num_poses))

    def calibrate(self, req):
        """
        Use AXXB to calibrate the camera frame.
        The calibrated pose is save at the data directory
        """

        self.data_dir = req.data_path
        self.load_transforms(self.data_dir)
        self.axxb()
        self.test()

        # Save the calibration file
        # Calibration file is in (x,y,z) and (x,y,z,w) yaml file
        calibration_file = os.path.join(req.data_path, 'camera_pose.yaml')
        calibration_rotm_file = os.path.join(req.data_path, 'camera_pose.txt')
        
        with open(calibration_rotm_file, 'w') as file1:
            for l in np.reshape(self.cam2ee, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        pose = dict()
        pose['cam_to_ee'] = dict()
        pose['cam_to_ee']['translation'] = dict()
        cam2ee_tran = np.array([self.cam2ee[0, -1], self.cam2ee[1, -1], self.cam2ee[2, -1]])
        cam2ee_tran = cam2ee_tran.tolist()
        pose['cam_to_ee']['translation']['x'] = cam2ee_tran[0]
        pose['cam_to_ee']['translation']['y'] = cam2ee_tran[1]
        pose['cam_to_ee']['translation']['z'] = cam2ee_tran[2]

        cam2ee_rotm = self.cam2ee[0:3, 0:3]

        assert cam2ee_rotm.shape == (3, 3)
        cam2ee_quat = rotm2quat(cam2ee_rotm)
        cam2ee_quat = cam2ee_quat.tolist()

        print (cam2ee_quat)

        pose['cam_to_ee']['quaternion'] = dict()
        pose['cam_to_ee']['quaternion']['w'] = cam2ee_quat[0]
        pose['cam_to_ee']['quaternion']['x'] = cam2ee_quat[1]
        pose['cam_to_ee']['quaternion']['y'] = cam2ee_quat[2]
        pose['cam_to_ee']['quaternion']['z'] = cam2ee_quat[3]

        with open(calibration_file, 'w') as outfile:
            yaml.dump(pose, outfile, default_flow_style=False)

        return CalibrateDataResponse("Successfully calibrate the data! The camera pose file is save in the arg directory.")

    def shutdown(self):
        self.robot.disconnect()

    def run_aruco_calibrate_server(self):
        s = rospy.Service("collect_data", CollectData, self.collect_data)
        # s = rospy.Service("collect_data_alvar", CollectData, self.collect_data_alvar)
        s = rospy.Service("calibrate_data", CalibrateData, self.calibrate)
        print ("Ready to calibrate!")

        rate = rospy.Rate(10)
        rospy.spin()

        rospy.on_shutdown(self.shutdown)
