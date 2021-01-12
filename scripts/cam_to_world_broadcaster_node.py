#!/usr/bin/python
import rospy
from calibration_toolbox.cam_to_world_broadcaster import Cam2WorldBroadcaster

if __name__ == "__main__":
    rospy.init_node("cam_to_world_tf_broadcaster")
    cam_to_ee_pose_file_path = "/home/hongtao/Dropbox/RSS2021/calib/1014_ps_rgb_ex_tool0/camera_pose.txt"
    CWB = Cam2WorldBroadcaster(cam_to_ee_pose_file_path, camera='ps')
    CWB.cam2world_broadcaster()


