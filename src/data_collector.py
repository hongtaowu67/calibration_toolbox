#! /usr/bin/python

import cv2
import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class RGBServer(object):
    
    def __init__(self):
        self.rgb_image_raw_topic = "/camera/rgb/image_raw"
        self.rgb_image_raw_sub = rospy.Subscriber(self.rgb_image_raw_topic, Image, self.rgb_image_raw_cb)
        
        self.cv_img = None
        self.bridge = CvBridge()

    def rgb_image_raw_cb(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.cv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2RGB)
        
        # Temporary for testing other function
        cv2.imwrite("/home/hongtao/Desktop/raw1.png", self.cv_img)

        print "Finish saving"
    

if __name__ == "__main__":
    rospy.init_node("capture_rgb_image_raw")
    RGBServer = RGBServer()
    
    rospy.spin()