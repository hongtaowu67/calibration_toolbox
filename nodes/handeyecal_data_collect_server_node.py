#! /usr/bin/python

import rospy
import time
from calibration_toolbox.handeyecal_data_collector import HandEyeCalDataCollector

if __name__ == "__main__":
    rospy.init_node("handeyecal_data_collect")

    time.sleep(0.5)

    HECDCServer = HandEyeCalDataCollector(camera='rs')
    HECDCServer.run_handeyecal_dta_collect_server()