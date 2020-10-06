from __future__ import print_function
import os
import cv2
import numpy as np
import yaml

from utils import *


axis = np.float32([[0, 0, 0], [5*0.0256,0,0], [0,5*0.0256,0], [0,0,5*0.0256]]).reshape(-1,3)

def draw(img, imgpts):
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),[0,0,255],3)  #BGR
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),[255,0,0],3)
    return img

def chessboard_pose(img_dir, img_filename, cam_mtx, cam_dist, objp, pattern=(7, 6)):
    img_filepath = os.path.join(img_dir, img_filename)
    img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)
    chessboard_size_tuple = pattern
    
    # IR and RGB both read as RGB and then turned to gray
    img = cv2.imread(img_filepath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(img, chessboard_size_tuple, None)

    if ret == True:
        # Increase corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        cv2.drawChessboardCorners(img, chessboard_size_tuple, corners, ret)
        # cv2.imwrite(os.path.join(img_dir, img_name + "_corner.png"), img)

        print ("Camera Info")
        print (cam_mtx)
        print (cam_dist)

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
        print ("imgpts")
        print (imgpts)

        img = draw(img, imgpts)
        cv2.imwrite(os.path.join(img_dir, img_name+ '_axis.png'),img)

        return R, tvecs
    else:
        print ("Cannot find chessboard in {}".format(img_filename))
        return None, None


if __name__ == "__main__":
    with open("/home/hongtao/.ros/camera_info/rgb_D435_RealSense.yaml", 'r') as f:
        doc = yaml.load(f)
        cam_mtx = np.array(doc['camera_matrix']['data']).reshape(3, 3)
        cam_dist = np.array(doc['distortion_coefficients']['data'])

    width = 0.0256
    x_num = 7
    y_num = 6
    objp = np.zeros((x_num * y_num, 3), np.float32)
    
    for i in range(y_num):
        for j in range(x_num):
            index = i * x_num + j
            objp[index, 0] = j * width
            objp[index, 1] = (y_num - 1 - i) * width
            objp[index, 2] = 0

    # data_dir = "/home/hongtao/Desktop"
    # fname = "rgb1.jpg"
    # R, t = chessboard_pose(data_dir, fname, cam_mtx, cam_dist, objp)
    # print (R)
    # print (t)

    data_dir = "/home/hongtao/Dropbox/RSS2021/calib/1005_extrinsic_calib_rs"
    file_list = os.listdir(data_dir)

    for fname in file_list:
        if "img.png" in fname:
            R, t = chessboard_pose(data_dir, fname, cam_mtx, cam_dist, objp)
            pose = make_rigid_transformation(t, R)

            img_idx = fname.split('_')[0]
            pose_file_path = os.path.join(data_dir, img_idx + '_markerpose.txt')
            # Tool pose in robot base frame
            with open(pose_file_path, 'w') as file1:
                for l in np.reshape(pose, (16, )).tolist():
                    file1.writelines(str(l) + ' ')