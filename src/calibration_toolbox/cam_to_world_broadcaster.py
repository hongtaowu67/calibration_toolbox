from __future__ import print_function

import numpy as np
import rospy

import tf2_ros
import geometry_msgs.msg
import utils


class Cam2WorldBroadcaster(object):

    def __init__(self, cam_to_ee_pose_file_path, camera='rs'):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        rospy.sleep(1.0)

        self.cam_root_name = "camera_link"

        self.ee_frame_name = "tool0_controller"
        if camera == 'rs':
            self.cam_frame_name = "camera_color_optical_frame"
        elif camera == 'ps':
            self.cam_frame_name = "camera_rgb_optical_frame"

        # root is the root of the camera tf tree
        root2cam = self.tfBuffer.lookup_transform(self.cam_frame_name, self.cam_root_name, rospy.Time(0))
        root2cam_pos_ros  = root2cam.transform.translation
        root2cam_quat_ros = root2cam.transform.rotation

        root2cam_pos  = np.array([root2cam_pos_ros.x, root2cam_pos_ros.y, root2cam_pos_ros.z])
        root2cam_quat = np.array([root2cam_quat_ros.w,
                                  root2cam_quat_ros.x,
                                  root2cam_quat_ros.y,
                                  root2cam_quat_ros.z])
        
        root2cam_rotm = utils.quat2rotm(root2cam_quat)
        root2cam_pose = utils.make_rigid_transformation(root2cam_pos, root2cam_rotm)
        print("root2cam pose")
        print(root2cam_pose)

        with open(cam_to_ee_pose_file_path, 'r') as f:
            cam2ee_str = f.readline()
            cam2ee = [float(x) for x in cam2ee_str.split()]
            cam2ee_pose = np.array(cam2ee).reshape(4, 4)
            print("cam2ee pose")
            print(cam2ee_pose)

        # TODO: Multiply root2cam_pose with cam2ee_pose and translate them to quaternion and translation
        root2ee = np.matmul(cam2ee_pose, root2cam_pose)
        
        self.pos = [0] * 3
        self.pos[0] = root2ee[0, -1]
        self.pos[1] = root2ee[1, -1]
        self.pos[2] = root2ee[2, -1]

        self.quat = utils.rotm2quat(root2ee[:3, :3])
 
        self.t = geometry_msgs.msg.TransformStamped()

        self.t.header.frame_id = self.ee_frame_name
        self.t.child_frame_id = self.cam_root_name
        self.t.transform.translation.x = self.pos[0]
        self.t.transform.translation.y = self.pos[1]
        self.t.transform.translation.z = self.pos[2]

        self.t.transform.rotation.w = self.quat[0]
        self.t.transform.rotation.x = self.quat[1]
        self.t.transform.rotation.y = self.quat[2]
        self.t.transform.rotation.z = self.quat[3]

    def cam2world_broadcaster(self):
        rate = rospy.Rate(10.0)

        rospy.loginfo("Start broadcasting cam2ee in tf")
        while not rospy.is_shutdown():

            self.t.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(self.t)          
            rate.sleep()


# cam_to_ee_pose_dict = utils.getDictFromYamlFilename(cam_to_ee_pose_file_path)

# self.quat = [0] * 4
# self.quat[0] = cam_to_ee_pose_dict['cam_to_ee']['quaternion']['w']
# self.quat[1] = cam_to_ee_pose_dict['cam_to_ee']['quaternion']['x']
# self.quat[2] = cam_to_ee_pose_dict['cam_to_ee']['quaternion']['y']
# self.quat[3] = cam_to_ee_pose_dict['cam_to_ee']['quaternion']['z']

# self.pos = [0] * 3
# self.pos[0] = cam_to_ee_pose_dict['cam_to_ee']['translation']['x']
# self.pos[1] = cam_to_ee_pose_dict['cam_to_ee']['translation']['y']
# self.pos[2] = cam_to_ee_pose_dict['cam_to_ee']['translation']['z']


