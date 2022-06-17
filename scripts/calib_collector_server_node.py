#! /usr/bin/python3

# Main script to run the data collector for calib_collector

import rospy

from calibration_toolbox.calib_collector import CalibrateCollector

if __name__ == "__main__":
    target = 'chessboard'
    calib_points_file = "/home/xin/Dropbox/SR2021_UC/220616_panda_ps_EH/calib_points.txt"
    image_topic = "/cam_1/rgb/image_raw"
    rospy.init_node("calib_collector_server", anonymous=True)
    CC = CalibrateCollector(target, calib_points_file, image_topic)
    CC.run()