#! /usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class BridgeTest():

    def __init__(self):
        
        rospy.init_node('BridgeTest_node', anonymous=False)

        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback)
    
    def camera_callback(self, data):

        # self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

        cv2.imshow('Image', self.image)
        cv2.waitKey(1)
    

if __name__ == "__main__":

    if not rospy.is_shutdown():
        BridgeTest()
        rospy.spin()