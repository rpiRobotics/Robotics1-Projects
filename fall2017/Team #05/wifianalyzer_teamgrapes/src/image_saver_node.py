#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils.jpg import image_cv_from_jpg

index = 0

def saveImage(image_msg):
    global index
    image = image_cv_from_jpg(image_msg.data)
    cv2.imwrite('./botimage%04d.png' % (index), image)
    index += 1

if __name__ == '__main__':
    rospy.init_node('image_saver_node', anonymous=False)
    rospy.Subscriber('/teamgrapes/camera_node/image/compressed', CompressedImage, saveImage)
    rospy.spin()
