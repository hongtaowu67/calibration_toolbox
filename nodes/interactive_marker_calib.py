#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy
import tf2_ros
import numpy as np
import os
import yaml

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from math import sin
from calibration_toolbox import utils

from calibration_toolbox.srv import *

server = None
menu_handler = MenuHandler()
br = None
counter = 0

class IntMarkerCalib(object):
    def __init__ (self, 
                  cam2ee_init_pose_file_path, 
                  parent_frame="tool0_controller",
                  cam_root_frame="camera_link",
                  cam_frame="camera_rgb_optical_frame"):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(1.0)

        self.parent_frame = parent_frame
        self.cam_frame = cam_frame
        self.cam_root_frame = cam_root_frame

        self.init_pos = None
        self.init_quat = None

        self.init_pos, self.init_quat, self.init_cam2ee_pose = self.getInitXform(cam2ee_init_pose_file_path)
        
        self.int_marker = InteractiveMarker()
        self.make6DofMarker( self.parent_frame, InteractiveMarkerControl.MOVE_ROTATE_3D )

        self.root2cam_pose = self.getRoot2Cam()

        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = parent_frame
        self.t.child_frame_id = cam_root_frame

        self.initRoot2EEXform()

        rospy.Timer(rospy.Duration(0.01), self.frameCallback)

    def frameCallback(self, msg):
        self.t.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(self.t)

    def initRoot2EEXform(self):
        init_root2ee_pose = np.matmul(self.init_cam2ee_pose, self.root2cam_pose)
        
        pos = np.zeros(3)
        pos = init_root2ee_pose[:3, -1]
        quat = utils.rotm2quat(init_root2ee_pose[:3, :3])

        self.t.transform.translation.x = pos[0]
        self.t.transform.translation.y = pos[1]
        self.t.transform.translation.z = pos[2]

        self.t.transform.rotation.w = quat[0]
        self.t.transform.rotation.x = quat[1]
        self.t.transform.rotation.y = quat[2]
        self.t.transform.rotation.z = quat[3]

    def getRoot2Cam(self):
        # root is the root of the camera tf tree
        root2cam = self.tfBuffer.lookup_transform(self.cam_frame, self.cam_root_frame, rospy.Time(0))
        root2cam_pos_ros  = root2cam.transform.translation
        root2cam_quat_ros = root2cam.transform.rotation

        root2cam_pos  = np.array([root2cam_pos_ros.x, root2cam_pos_ros.y, root2cam_pos_ros.z])
        root2cam_quat = np.array([root2cam_quat_ros.w,
                                  root2cam_quat_ros.x,
                                  root2cam_quat_ros.y,
                                  root2cam_quat_ros.z])
        
        root2cam_rotm = utils.quat2rotm(root2cam_quat)
        root2cam_pose = utils.make_rigid_transformation(root2cam_pos, root2cam_rotm)

        return root2cam_pose

    def getInitXform(self, cam2ee_pose_file_path):
        """
        Get the initial xform between cam to ee
        
        Args:
            - cam2ee_pose_file_path: path the cam to ee file (txt)
        Returns:
            - cam2ee_pos ((3, ) numpy array): position
            - cam2ee_quat ((4, ) numpy array): quaternion (w, x, y, z)
        """
        with open(cam2ee_pose_file_path, 'r') as f:
            cam2ee_str = f.readline()
            cam2ee = [float(x) for x in cam2ee_str.split()]
            cam2ee_pose = np.array(cam2ee).reshape(4, 4)
            print("cam2ee pose")
            print(cam2ee_pose)

        cam2ee_pos  = cam2ee_pose[:3, -1]
        cam2ee_quat = utils.rotm2quat(cam2ee_pose[:3, :3])
        
        return cam2ee_pos, cam2ee_quat, cam2ee_pose

    def processFeedback(self, feedback):
        cam2ee_pos_ros = feedback.pose.position
        cam2ee_quat_ros = feedback.pose.orientation

        cam2ee_pos = np.array([cam2ee_pos_ros.x, cam2ee_pos_ros.y, cam2ee_pos_ros.z])
        cam2ee_quat = np.array([cam2ee_quat_ros.w, cam2ee_quat_ros.x, cam2ee_quat_ros.y, cam2ee_quat_ros.z])

        cam2ee_rotm = utils.quat2rotm(cam2ee_quat)
        cam2ee_pose = utils.make_rigid_transformation(cam2ee_pos, cam2ee_rotm)

        root2ee_pose = np.matmul(cam2ee_pose, self.root2cam_pose)

        pos = np.zeros(3)
        pos = root2ee_pose[:3, -1]
        quat = utils.rotm2quat(root2ee_pose[:3, :3])

        self.t.transform.translation.x = pos[0]
        self.t.transform.translation.y = pos[1]
        self.t.transform.translation.z = pos[2]

        self.t.transform.rotation.w = quat[0]
        self.t.transform.rotation.x = quat[1]
        self.t.transform.rotation.y = quat[2]
        self.t.transform.rotation.z = quat[3]

        server.applyChanges()

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def makeBoxControl(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    def make6DofMarker(self, parent_frame, interaction_mode):

        self.int_marker.header.frame_id = parent_frame
        
        self.int_marker.pose.position.x = self.init_pos[0]
        self.int_marker.pose.position.y = self.init_pos[1]
        self.int_marker.pose.position.z = self.init_pos[2]

        self.int_marker.pose.orientation.w = self.init_quat[0]
        self.int_marker.pose.orientation.x = self.init_quat[1]
        self.int_marker.pose.orientation.y = self.init_quat[2]
        self.int_marker.pose.orientation.z = self.init_quat[3]

        self.int_marker.scale = 0.2

        # insert a box
        self.makeBoxControl(self.int_marker)
        self.int_marker.controls[0].interaction_mode = interaction_mode


        control_modes_dict = { 
                        InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                        InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                        InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        self.int_marker.name += "_" + control_modes_dict[interaction_mode]
        self.int_marker.description = "3D Control"
        self.int_marker.description += " + 6-DOF controls"
        self.int_marker.description += "\n" + control_modes_dict[interaction_mode]
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.int_marker.controls.append(control)

        server.insert(self.int_marker, self.processFeedback)
        menu_handler.apply( server, self.int_marker.name )

    def save_refined_xform(self, req):
        calibration_yaml = os.path.join(req.data_dir, "camera_pose_ref.yaml")
        calibration_txt  = os.path.join(req.data_dir, "camera_pose_ref.txt")

        cam2ee_pose_ros = self.tfBuffer.lookup_transform(self.parent_frame, self.cam_frame, rospy.Time(0))

        cam2ee_pos_ros = cam2ee_pose_ros.transform.position
        cam2ee_quat_ros = cam2ee_pose_ros.transform.rotation

        cam2ee_pos  = np.array([cam2ee_pos_ros.x, cam2ee_pos_ros.y, cam2ee_pos_ros.z])
        cam2ee_quat = np.array([cam2ee_quat_ros.w, cam2ee_quat_ros.x, cam2ee_quat_ros.y, cam2ee_quat_ros.z])

        cam2ee_rotm = utils.quat2rotm(cam2ee_quat)
        cam2ee_pose = utils.make_rigid_transformation(cam2ee_pos, cam2ee_rotm)

        with open(calibration_txt, 'w') as file1:
            for l in np.reshape(cam2ee_pose, (16, )).tolist():
                file1.writelines(str(l) + ' ')

        pose = dict()
        pose['cam_to_ee'] = dict()
        pose['cam_to_ee']['translation'] = dict()
        pose['cam_to_ee']['translation']['x'] = cam2ee_pos[0]
        pose['cam_to_ee']['translation']['y'] = cam2ee_pos[1]
        pose['cam_to_ee']['translation']['z'] = cam2ee_pos[2]

        pose['cam_to_ee']['quaternion'] = dict()
        pose['cam_to_ee']['quaternion']['w'] = cam2ee_quat[0]
        pose['cam_to_ee']['quaternion']['x'] = cam2ee_quat[1]
        pose['cam_to_ee']['quaternion']['y'] = cam2ee_quat[2]
        pose['cam_to_ee']['quaternion']['z'] = cam2ee_quat[3]

        with open(calibration_yaml, 'w') as outfile:
            yaml.dump(pose, outfile, default_flow_style=False)

        rospy.loginfo("Finish saving at {}".format(calibration_yaml))


if __name__=="__main__":

    rospy.init_node("basic_controls")
    br = TransformBroadcaster()

    server = InteractiveMarkerServer("basic_controls")  
    
    cam2ee_init_pose_file_path = "/home/hongtao/Dropbox/RSS2021/calib/1014_ps_rgb_ex_tool0/camera_pose.txt"
    IMC = IntMarkerCalib(cam2ee_init_pose_file_path)

    parent_frame = "tool0_controller"
    
    IMC.make6DofMarker( parent_frame, InteractiveMarkerControl.MOVE_ROTATE_3D )
    server.applyChanges()

    s = rospy.Service("save_ref_xform", IntMarkerCalib, IMC.save_refined_xform)

    rospy.spin()