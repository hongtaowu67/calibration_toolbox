"""
Get the projection matrix and write the camera_info yaml file for ros
Please copy and paste the data to the yaml file
TODO: write code to generate yaml file
"""
import cv2
import yaml

import numpy as np

yaml_file_path = "/home/hongtao/Desktop/rgb_PS1080_PrimeSense_test.yaml"

camera_name = "rgb_PS1080_PrimeSense"
camera_K = [536.6116813173013, 0.0, 320.4797586670793, 0.0, 536.1139350566275, 233.8974830065129, 0.0, 0.0, 1.0]
camera_D = [0.04322271228851884, -0.1286752521716093, -0.001478034743738911, 0.0003779828219621169, 0.0]
image_height = 480
image_width = 640
camera_R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
dist_model = 'plumb_bob'

camera_K_np = np.array(camera_K).reshape(3, 3)
camera_D_np = np.array(camera_D)
camera_R_np = np.array(camera_R).reshape(3, 3)
camera_size = (image_width, image_height)

camera_P = cv2.getOptimalNewCameraMatrix(camera_K_np, camera_D_np, camera_size, 0.0)

print ("Camera P")
camera_P = camera_P[0]
last_col = np.array([0.0, 0.0, 0.0])
camera_P = np.c_[camera_P, last_col]
camera_P = camera_P.flatten().tolist()
print (camera_P)