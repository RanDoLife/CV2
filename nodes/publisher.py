#!/usr/bin/env python3
# encoding: utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"
ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"


def generate_image(width=320, height=240) -> Image:
    """
    Генерирует случайное изображение заданного разрешения.
    Формат пикселей: монохром, 8-бит (mono8).
    """
    bridge = CvBridge()
    random_image = np.random.randint(0, 256, (height, width), dtype=np.uint8)
    ros_image = bridge.cv2_to_imgmsg(random_image, encoding='mono8')
    return ros_image


def main() -> None:
    rospy.init_node(ROS_NODE_NAME)
    pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)
    publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

    rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

    rate = rospy.Rate(pub_frequency)
    while not rospy.is_shutdown():
        image_msg = generate_image()
        publisher.publish(image_msg)
        rate.sleep()


if __name__ == '__main__':
    main()
