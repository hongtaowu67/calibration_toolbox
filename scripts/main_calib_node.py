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
    data_dir = "/home/raya/Dropbox/140821_panda_ps_EBME"

    # Calibration option
    #"EBCB" (eye-on-base, get camera to base), 
    #"EBME" (eye-on-base, get marker in ee), 
    #"EH" (eye-on-hand, get ee to camera)
    option = "EBME"

    # Relative transformation
    # Use None if there are no relative transformation
    # between the target frame and the marker frame
    relative_xform = None
    # relative_xform = np.array([
    #     [1.0, 0.0, 0.0, 0.0],
    #     [0.0, 1.0, 0.0, 0.0],
    #     [0.0, 0.0, 0.0, -0.040],
    #     [0.0, 0.0, 0.0, 1.0]
    # ])

    AXXBCalib = AXXBCalibrator(option)
    AXXBCalib.load_xforms(data_dir, relative_xform)

    cam2ee = AXXBCalib.axxb()

    AXXBCalib.test()
    AXXBCalib.write_pose_file()