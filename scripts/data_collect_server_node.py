#! /usr/bin/python

import rospy
import time
from calibration_toolbox.data_collector import DataCollectServer

if __name__ == "__main__":
    rospy.init_node("capture_rgb_ir")

    time.sleep(0.5)

    DCServer = DataCollectServer()
    DCServer.run_data_collector_server()