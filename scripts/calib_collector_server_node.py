#! /usr/bin/python

# Main script to run the data collector for calib_collector

import rospy

from calibration_toolbox.calib_collector import CalibrateCollector

if __name__ == "__main__":
    target = 'chessboard'
    calib_points_file = "/home/raya/Dropbox/180821_panda_ps_EBME/calib_points.txt"
    image_topic = "/camera/rgb/image_raw"
    rospy.init_node("calib_collector_server", anonymous=True)
    CC = CalibrateCollector(target, calib_points_file, image_topic)
    CC.run()