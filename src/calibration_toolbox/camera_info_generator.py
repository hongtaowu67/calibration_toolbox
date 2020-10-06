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
camera_K = [537.913865361402600, 0.0, 318.935253479989512, 0.0, 537.663890471360560, 233.440991108226967, 0.0, 0.0, 1.0]
camera_D = [0.048090838215926, -0.144203330639063, 0.000109679993889, -0.000707301501196, 0.000000000000000]
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