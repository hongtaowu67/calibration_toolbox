#! /usr/bin/python
import rospy
import sys

from aruco_calib.aruco_calibrate import Calibrate

rospy.init_node("aruco_calibration_server", anonymous=True)
C = Calibrate(calib_point_num=25)
C.run_aruco_calibrate_server()