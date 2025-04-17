#!/usr/bin/env python3
# encoding: utf-8

import cv2

import rospy

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image


bridge = CvBridge()

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
    cv2.imshow("Received Image In RGB24", cv_image)
    cv2.waitKey(1)  # Важно для обновления окна


def formate():
    rospy.init_node('reformator')

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    
    rospy.spin()

if __name__ == "__main__":
    formate()