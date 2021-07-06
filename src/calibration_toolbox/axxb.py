# AXXB solver
# Author: Hongtao Wu
# Johns Hopkins University
# Apr 09, 2021

from __future__ import print_function, division
import numpy as np
import os
import yaml

from utils import *

class AXXBCalibrator(object):
    """
    Hand-eye calibration for AXXB problem
    """
    def __init__(self, option="EH"):
        """
        @type  option: string
        @param option: type of camera-robot configuration. 
                "EBCB" (eye-on-base, get camera to base), 
                "EBME" (eye-on-base, get marker in ee), 
                "EH" (eye-on-hand, get ee to camera)
        """
        self.robot_poses = []
        self.marker_poses = []
        self.pose_indices = []
        self.calib_pose = None

        self.data_dir = None

        self.option = option
        self.option_list = ["EBCB", "EBME", "EH"]
        assert self.option in self.option_list, "Option should be EBCB, EBME, or EH!"

        if self.option is "EBCB":
            self.option_str = "cam_to_base"
        elif self.option is "EBME":
            self.option_str = "marker_to_ee"
        elif self.option is "EH":
            self.option_str = "cam_to_ee"
    
    def load_xforms(self, load_dir):
        """ 
        Load robot pose and marker pose from a save directory
        Two lists of 4x4 homogeneous transformatin matrices will be saved in self.robot_poses
        and self.marker_poses
        
        @type  load_dir: string
        @param load_dir: the directory where calibration data was previously acquired from the robot.
        """
        self.data_dir = load_dir
        print ("Loading {}...".format(load_dir))
        for f in os.listdir(load_dir):
            if 'robotpose.txt' in f:
                print ("Loading {}...".format(f))

                pose_idx = int(f.split('_')[0])
                self.pose_indices.append(pose_idx)

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
        
        @rtype  self.calib_pose: 4x4 numpy.ndarry
        @return self.calib_pose: target poses.
        """
        assert len(self.robot_poses) == len(self.marker_poses), 'robot poses and marker poses are not of the same length!'

        n = len(self.robot_poses)
        pose_inds= np.arange(n)
        np.random.shuffle(pose_inds)

        print ("Total Pose: %i" % n)
        A = np.zeros((4, 4, n-1))
        B = np.zeros((4, 4, n-1))
        alpha = np.zeros((3, n-1))
        beta = np.zeros((3, n-1))

        M = np.zeros((3, 3))

        for i in range(n-1):
            if self.option == "EH":
                A[:, :, i] = np.matmul(pose_inv(self.robot_poses[pose_inds[i+1]]), self.robot_poses[pose_inds[i]])
                B[:, :, i] = np.matmul(self.marker_poses[pose_inds[i+1]], pose_inv(self.marker_poses[pose_inds[i]]))
            elif self.option == "EBME":
                A[:, :, i] = np.matmul(pose_inv(self.robot_poses[pose_inds[i+1]]), self.robot_poses[pose_inds[i]])
                B[:, :, i] = np.matmul(pose_inv(self.marker_poses[pose_inds[i+1]]), self.marker_poses[pose_inds[i]])
            elif self.option == "EBCB":
                A[:, :, i] = np.matmul(self.robot_poses[pose_inds[i+1]], pose_inv(self.robot_poses[pose_inds[i]]))
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
        
        self.calib_pose = np.c_[R, t]
        self.calib_pose = np.r_[self.calib_pose, [[0, 0, 0, 1]]]

        print ("Calibration Result:\n", self.calib_pose)

        return self.calib_pose

    def test(self):
        """ 
        Test the accuracy of the calculated result.
        Use AXB to calculate the check_pose transformation for each frame.
        """
        n = len(self.robot_poses)
        for i in range(n):
            if self.option == "EH":
                check_pose = np.matmul(np.matmul(self.robot_poses[i], self.calib_pose), self.marker_poses[i])
            elif self.option == "EBME":
                check_pose = np.matmul(np.matmul(self.robot_poses[i], self.calib_pose), pose_inv(self.marker_poses[i]))
            elif self.option == "EBCB":
                check_pose = np.matmul(np.matmul(pose_inv(self.robot_poses[i]), self.calib_pose), self.marker_poses[i])

            print("check_pose #{}".format(self.pose_indices[i]))
            print(check_pose)
            print("=========")
    
    def write_pose_file(self):
        """
        Save the calibration file
        Calibration file is in (x,y,z) and (x,y,z,w) yaml file
        """
        calibration_file = os.path.join(self.data_dir, 'pose.yaml')
        calibration_rotm_file = os.path.join(self.data_dir, 'pose.txt')
        
        with open(calibration_rotm_file, 'w') as file1:
            for l in np.reshape(self.calib_pose, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        pose = dict()
        pose[self.option_str] = dict()
        pose[self.option_str]['translation'] = dict()
        calib_pose_tran = np.array([self.calib_pose[0, -1], self.calib_pose[1, -1], self.calib_pose[2, -1]])
        calib_pose_tran = calib_pose_tran.tolist()
        pose[self.option_str]['translation']['x'] = calib_pose_tran[0]
        pose[self.option_str]['translation']['y'] = calib_pose_tran[1]
        pose[self.option_str]['translation']['z'] = calib_pose_tran[2]

        calib_pose_rotm = self.calib_pose[0:3, 0:3]

        assert calib_pose_rotm.shape == (3, 3)
        calib_pose_quat = rotm2quat(calib_pose_rotm)
        calib_pose_quat = calib_pose_quat.tolist()

        print (calib_pose_quat)

        pose[self.option_str]['quaternion'] = dict()
        pose[self.option_str]['quaternion']['w'] = calib_pose_quat[0]
        pose[self.option_str]['quaternion']['x'] = calib_pose_quat[1]
        pose[self.option_str]['quaternion']['y'] = calib_pose_quat[2]
        pose[self.option_str]['quaternion']['z'] = calib_pose_quat[3]

        with open(calibration_file, 'w') as outfile:
            yaml.dump(pose, outfile, default_flow_style=False)



