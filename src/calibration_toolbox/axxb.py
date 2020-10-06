from __future__ import print_function, division
import numpy as np
import os
import yaml

from utils import *

class AXXBCalibrator(object):
    def __init__(self):
        self.robot_poses = []
        self.marker_poses = []
        self.pose_indices = []
        self.cam2ee = None

        self.data_dir = None
    
    def load_xforms(self, load_dir):
        """ Load robot pose and marker pose from a save directory
        # Arguments
        load_dir: the directory where calibration data was previously acquired from the robot.
        # Returns
        Two lists of 4x4 transformation matrices.
        self.robot_poses, self.marker_poses
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
        
        Args:
        - self.robot_poses (list of 4x4 numpy array): poses (homogenous transformation) of the robot end-effector in the robot base frame.
        - self.marker_poses (list of 4x4 numpy array): poses (homogenous transformation) of the marker in the camera frame.
        Return:
        - self.cam2ee (4x4 numpy array): poses of the camera in the robot end-effector frame.
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

        return self.cam2ee

    def test(self):
        """ Test the accuracy of the calculated result.
            Use AXB to calculate the base2tag transformation for each frame.
        """
        n = len(self.robot_poses)
        for i in range(n):
            base2tag = np.matmul(np.matmul(self.robot_poses[i], self.cam2ee), self.marker_poses[i])
            print("base2tag #{}".format(self.pose_indices[i]))
            print(base2tag)
            # print("cam2tag #{}".format(self.pose_indices[i]))
            # print(self.robot_poses[i])
            print("=========")
    
    def write_pose_file(self):
        # Save the calibration file
        # Calibration file is in (x,y,z) and (x,y,z,w) yaml file
        calibration_file = os.path.join(self.data_dir, 'camera_pose.yaml')
        calibration_rotm_file = os.path.join(self.data_dir, 'camera_pose.txt')
        
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




if __name__ == "__main__":
    data_dir = "/home/hongtao/Dropbox/RSS2021/calib/1006_extrinsic_calib_ps_tool0"
    
    AXXBCalib = AXXBCalibrator()
    AXXBCalib.load_xforms(data_dir)

    cam2ee = AXXBCalib.axxb()

    AXXBCalib.test()
    AXXBCalib.write_pose_file()
