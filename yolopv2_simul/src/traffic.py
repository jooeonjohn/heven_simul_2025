#! /usr/bin/env python3

import rospy, cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from cv_bridge import CvBridge

class LaneDetect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('traffic_node', anonymous=False)

        self.image_orig = None
        self.pose_x = 0
        self.pose_y = 0

        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_orig_callback)
        rospy.Subscriber('/mm_pose_parser/pose', Pose2D, self.pose_callback)
        self.traffic_pub = rospy.Publisher('/traffic_signal', Int32, queue_size=1)

    def pose_callback(self, data):
        """Callback function for /mm_pose_parser/pose topic."""
        self.pose_x = data.x
        self.pose_y = data.y
        rospy.loginfo(f"Received pose: x={self.pose_x}, y={self.pose_y}")

    def camera_orig_callback(self, data):
        """Callback function for camera image."""
        self.image_orig = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        traffic_signal = self.traffic_detect()
        self.traffic_pub.publish(traffic_signal)


    def define_roi(self):
        """Define ROI based on specific pose cases."""
        if abs(self.pose_x - 51.03) < 5.0 and abs(self.pose_y - (-161.14)) < 5.0:
            roi_coords = (220, 120, 450, 250)  # ROI for case 1
        elif abs(self.pose_x - 29.31) < 5.0 and abs(self.pose_y - (-16.61)) < 5.0:
            roi_coords = (290, 100, 520, 230)  # ROI for case 2
        elif abs(self.pose_x - (-43.94)) < 5.0 and abs(self.pose_y - (-17.01)) < 5.0:
            roi_coords = (190, 260, 340, 340)  # ROI for case 3
        elif abs(self.pose_x - (-93.85)) < 5.0 and abs(self.pose_y - 98.52) < 5.0:
            roi_coords = (220, 120, 450, 250)  # ROI for case 4
        else:
            roi_coords = (0, 0, self.image_orig.shape[1], self.image_orig.shape[0])  # Default to the entire image

        # Draw the ROI on the original image
        x1, y1, x2, y2 = roi_coords
        cv2.rectangle(self.image_orig, (x1, y1), (x2, y2), (0, 0, 0), 2)  # Green rectangle with thickness 2

        return self.image_orig[y1:y2, x1:x2]  # Return the cropped ROI

    def color_filter_red(self, image):
        """Filter red color."""
        lower = np.array([0, 0, 100])
        upper = np.array([100, 100, 255])
        mask = cv2.inRange(image, lower, upper)
        return mask

    def color_filter_green(self, image):
        """Filter green color."""
        lower = np.array([0, 100, 0])
        upper = np.array([100, 255, 100])
        mask = cv2.inRange(image, lower, upper)
        return mask

    def traffic_detect(self):
        """Detect traffic light color."""
        if self.image_orig is None:
            return -1  # No image received yet

        roi = self.define_roi()

        # Apply color filters
        red_mask = self.color_filter_red(roi)
        green_mask = self.color_filter_green(roi)

        # Display the image with ROI
        cv2.imshow("Traffic Image with ROI", self.image_orig)
        # cv2.imshow("Red Mask", red_mask)
        # cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)  # Add a small delay to allow the images to render

        # Count non-zero pixels in the masks
        red_count = cv2.countNonZero(red_mask)
        green_count = cv2.countNonZero(green_mask)

        print(f"Red count: {red_count}, Green count: {green_count}")

        if red_count > green_count:
            rospy.loginfo("Red light detected.")
            return 1  # Red light
        elif green_count > red_count:
            rospy.loginfo("Green light detected.")
            return 0  # Green light
        else:
            rospy.loginfo("No significant color detected.")
            return -1  # No significant color detected




if __name__ == "__main__":
    try:
        LaneDetect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass