#!/usr/bin/env python3
# encoding: utf-8

import cv2
import rospy
import numpy as np

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image

bridge = CvBridge()
prev_frame = None  # Глобальная переменная для хранения предыдущего кадра

def callback(msg):
    global prev_frame  # Объявляем, что будем изменять глобальную переменную
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Преобразуем в оттенки серого
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Если есть предыдущий кадр, вычисляем разницу
    if prev_frame is not None:
        diff = cv2.absdiff(gray, prev_frame)
        if np.sum(diff) > 500000:  # Порог чувствительности
            rospy.loginfo("Motion detected!")

    # Обновляем предыдущий кадр
    prev_frame = gray.copy()

    # Отображаем изображение (опционально)
    cv2.imshow("Camera Feed", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('motion_detector', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    main()