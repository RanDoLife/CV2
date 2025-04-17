#!/usr/bin/env python3
# encoding: utf-8

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
image_pub = None
mask_pub = None

def get_hsv_range(color_name):
    if color_name == "red":
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 100, 100])
        upper2 = np.array([180, 255, 255])
        return [(lower1, upper1), (lower2, upper2)]
    elif color_name == "green":
        lower = np.array([50, 100, 100])
        upper = np.array([70, 255, 255])
        return [(lower, upper)]
    elif color_name == "blue":
        lower = np.array([100, 100, 100])
        upper = np.array([130, 255, 255])
        return [(lower, upper)]
    else:
        return []

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    target_color = rospy.get_param("/target_color", "red")
    color_ranges = get_hsv_range(target_color)

    mask = None
    for lower, upper in color_ranges:
        current_mask = cv2.inRange(hsv_image, lower, upper)
        mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)

    if mask is not None and cv2.countNonZero(mask) > 0:
        rospy.loginfo(f"Обнаружен цвет: {target_color}")
    else:
        rospy.loginfo("Цвет не найден")

    # Публикация исходного изображения
    image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    image_pub.publish(image_msg)

    # Публикация маски (одноканальное изображение)
    mask_msg = bridge.cv2_to_imgmsg(mask, encoding="mono8")
    mask_pub.publish(mask_msg)

def main():
    global image_pub, mask_pub
    rospy.init_node('image_filter', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    # Инициализация паблишеров
    image_pub = rospy.Publisher("/filtered/image_raw", Image, queue_size=1)
    mask_pub = rospy.Publisher("/filtered/mask", Image, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
