from __future__ import print_function
import cv2
import numpy as np
import yaml


axis = np.float32([[0, 0, 0], [5*0.0256,0,0], [0,5*0.0256,0], [0,0,5*0.0256]]).reshape(-1,3)

def draw(img, imgpts):
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),[255,0,0],2)  #BGR
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],2)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),[0,0,255],2)
    return img

def chessboard_pose(img_filename, cam_mtx, cam_dist, objp, pattern=(7, 6)):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    
    chessboard_size_tuple = pattern
    

    img = cv2.imread(img_filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(img, chessboard_size_tuple, None)

    if ret == True:
        # Increase corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        cv2.drawChessboardCorners(img, chessboard_size_tuple, corners, ret)
        cv2.imwrite(img_filename + ".png", img)

        print ("Camera Info")
        print (cam_mtx)
        print (cam_dist)

        _, rvecs, tvecs, inlier = cv2.solvePnPRansac(objp, corners2, cam_mtx, cam_dist)
        
        print ("aa")
        print (rvecs)
        print ("t")
        print (tvecs)

        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)

        imgpts = np.int32(imgpts).reshape(-1,2)
        print ("imgpts")
        print (imgpts)

        img = draw(img, imgpts)
        cv2.imwrite(img_filename + '.1.png',img)



if __name__ == "__main__":
    with open("/home/hongtao/.ros/camera_info/rgb_PS1080_PrimeSense.yaml", 'r') as f:
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


    img_filename = "/home/hongtao/Desktop/raw1.png"
    chessboard_pose(img_filename, cam_mtx, cam_dist, objp)