#! /usr/bin/python

import rospy

from calibration_toolbox.aruco_calib_collector import ArucoCalibrateCollector

if __name__ == "__main__":
    rospy.init_node("aruco_calib_collector_server", anonymous=True)
    ACC = ArucoCalibrateCollector(calib_point_num=25)
    ACC.run()