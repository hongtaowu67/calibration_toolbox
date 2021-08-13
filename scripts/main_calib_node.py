#! /usr/bin/python

# Main script for AXXB calibration
# Author: Hongtao Wu
# Johns Hopkins University
# July 06, 2021

import rospy

from calibration_toolbox.axxb import AXXBCalibrator
from calibration_toolbox.utils import *

if __name__ == "__main__":
    rospy.init_node("main_calib_node", anonymous=True)
    
    # Data directory for saving the captured data
    data_dir = "//home/raya/Dropbox/070821_panda_ps_ebme"

    # Calibration option
    #"EBCB" (eye-on-base, get camera to base), 
    #"EBME" (eye-on-base, get marker in ee), 
    #"EH" (eye-on-hand, get ee to camera)
    option = "EBME"

    AXXBCalib = AXXBCalibrator(option)
    AXXBCalib.load_xforms(data_dir)

    cam2ee = AXXBCalib.axxb()

    AXXBCalib.test()
    AXXBCalib.write_pose_file()