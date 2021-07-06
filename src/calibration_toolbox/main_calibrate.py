#! /usr/bin/python

# Main script for calibration
# Author: Hongtao Wu
# Johns Hopkins University
# July 06, 2021

from axxb import AXXBCalibrator
from utils import *

if __name__ == "__main__":
    # Data directory for saving the captured data
    data_dir = "/home/hongtao/Desktop/041321_panda_ps"

    # Calibration option
    #"EBCB" (eye-on-base, get camera to base), 
    #"EBME" (eye-on-base, get marker in ee), 
    #"EH" (eye-on-hand, get ee to camera)
    option = "EH"

    AXXBCalib = AXXBCalibrator(option)
    AXXBCalib.load_xforms(data_dir)

    cam2ee = AXXBCalib.axxb()

    AXXBCalib.test()
    AXXBCalib.write_pose_file()