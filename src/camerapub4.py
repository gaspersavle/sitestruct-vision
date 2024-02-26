#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def usb_camera_publisher():
    # Initialize ROS node
    rospy.init_node('camera_pub4', anonymous=True)

    # Initialize OpenCV video capture
    cap = cv2.VideoCapture("/dev/video1")  # Assuming USB camera is at index 0
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    if not cap.isOpened():
        rospy.logerr("Failed to open USB camera [4].")
        return

    # Initialize ROS publisher
    pub = rospy.Publisher('cam4/image_raw/compressed', CompressedImage, queue_size=10)
    rate = rospy.Rate(30)  # Publish rate (30 Hz)

    # Initialize CvBridge
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert OpenCV image to ROS image
            ros_image = bridge.cv2_to_compressed_imgmsg(frame)
            # Publish ROS image
            pub.publish(ros_image)
        rate.sleep()

    # Release resources
    cap.release()

if __name__ == '__main__':
    try:
        usb_camera_publisher()
    except rospy.ROSInterruptException:
        pass

