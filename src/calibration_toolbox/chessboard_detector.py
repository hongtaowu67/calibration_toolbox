# Detecting chessboard in the data
# Author: Hongtao Wu
# Johns Hopkins University
# Date: Apr 09, 2021

from __future__ import print_function
import os
import cv2
import numpy as np
import yaml

from utils import *

# Camera info
camera_info_yaml = "/home/raya/.ros/camera_info/rgb_PS1080_PrimeSense.yaml"

# Chessboard pattern
width = 0.029
x_num = 5
y_num = 7

# Data directory for saving the calibration data
data_dir = "/home/raya/Dropbox/070821_panda_rs_eh_down"

# Visualized axis on the data
axis = np.float32([[0, 0, 0], [3*width,0,0], [0,3*width,0], [0,0,3*width]]).reshape(-1,3)

def draw(img, imgpts):
    """
    Draw the detction result (axis & origin of teh chessboard) on the image
    """
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),[0,0,255],3)  #BGR
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),[255,0,0],3)
    return img

def chessboard_pose(img_dir, img_filename, cam_mtx, cam_dist, objp, pattern=(7, 6)):
    """
    Find the chessboard pose with OpenCV.

    @type  img_dir: string
    @param img_dir: directory of the image
    @type  img_filename: string
    @param img_filename: filename of the image
    @type  cam_mtx: numpy.ndarray
    @param cam_mtx: intrinsic matrix of the camera
    @type  cam_dist: numpy.ndarry
    @param cam_dist: distortion coefficient of the camera
    @type  objp: numpy.ndarray
    @param objp: 3D positions of the points on the chessboard
    @type  pattern: tuple
    @param pattern: pattern of the chessboard
    """
    img_filepath = os.path.join(img_dir, img_filename)
    img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)
    chessboard_size_tuple = pattern
    
    # IR and RGB both read as RGB and then turned to gray
    img = cv2.imread(img_filepath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size_tuple, None)

    if ret == True:
        # Increase corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        cv2.drawChessboardCorners(img, chessboard_size_tuple, corners, ret)
        cv2.imwrite(os.path.join(img_dir, img_name + "_corner.png"), img)

        # print ("Camera Info")
        # print (cam_mtx)
        # print (cam_dist)

        _, rvecs, tvecs, inlier = cv2.solvePnPRansac(objp, corners2, cam_mtx, cam_dist)
        
        # print ("aa")
        # print (rvecs)
        # print ("t")
        # print (tvecs)

        R, _ = cv2.Rodrigues(rvecs)
        # print ("rotm")
        # print (R)

        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)

        imgpts = np.int32(imgpts).reshape(-1,2)
        # print ("imgpts")
        # print (imgpts)

        img = draw(img, imgpts)
        cv2.imwrite(os.path.join(img_dir, img_name+ '_axis.png'),img)

        print ("Finish processing: {}".format(img_filename))

        return R, tvecs
    else:
        print ("Cannot find chessboard in {}".format(img_filename))
        return None, None


if __name__ == "__main__":
    # with open(camera_info_yaml, 'r') as f:
    #     doc = yaml.load(f)
    #     cam_mtx = np.array(doc['camera_matrix']['data']).reshape(3, 3)
    #     cam_dist = np.array(doc['distortion_coefficients']['data'])
    cam_mtx = np.array([615.2091064453125, 0.0, 325.8149719238281, 0.0, 615.3001708984375, 236.04461669921875, 0.0, 0.0, 1.0])
    cam_dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    cam_mtx = cam_mtx.reshape(3, 3)

    print ("cam_mtx")
    print (cam_mtx)
    print("------------")

    print ("cam_dist")
    print (cam_dist)
    print("------------")

    pattern = (x_num, y_num)
    objp = np.zeros((x_num * y_num, 3), np.float32)
    
    for i in range(y_num):
        for j in range(x_num):
            index = i * x_num + j
            objp[index, 0] = j * width
            objp[index, 1] = i * width
            objp[index, 2] = 0

    file_list = os.listdir(data_dir)

    for fname in file_list:
        if ".png" in fname:
            print (fname)
            R, t = chessboard_pose(data_dir, fname, cam_mtx, cam_dist, objp, pattern)
            pose = make_rigid_transformation(t, R)

            img_idx = fname.split('_')[0]
    
            pose_file_path = os.path.join(data_dir, img_idx + '_markerpose.txt')
            # Tool pose in robot base frame
            with open(pose_file_path, 'w') as file1:
                for l in np.reshape(pose, (16, )).tolist():
                    file1.writelines(str(l) + ' ')