#! /usr/bin/python

# Main script to run the data collector for calib_collector

import rospy

from calibration_toolbox.calib_collector import CalibrateCollector

if __name__ == "__main__":
    target = 'chessboard'
    calib_points_file = "/home/raya/Dropbox/160921_panda_ps_EH_scan/calib_points.txt"
    # image_topic = "/camera/color/image_raw" # Realsense
    image_topic = "/cam_1/rgb/image_raw" # Primesense
    base_frame_name = "panda_link0"
    ee_frame_name = "panda_hand"
    # camera_frame_name = "camera_color_optical_frame"
    camera_frame_name = "came_1_rgb_optical_frame"

    rospy.init_node("calib_collector_server", anonymous=True)
    CC = CalibrateCollector(target, calib_points_file, image_topic, 
                            base_frame=base_frame_name,
                            ee_frame=ee_frame_name,
                            camera_frame=camera_frame_name)
    CC.run()