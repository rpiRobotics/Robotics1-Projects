#!/usr/bin/env python

"""
Author: Zac Ravichandran
Node to process camera frames and publish found obstacles
"""

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from duckietown_msgs.msg import AprilTagDetection 
from duckietown_msgs.msg import AprilTagDetectionArray
from sign_detector import find_stop_signs, find_ducks
import numpy as np
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
import cv2
import os 
import fit_svm

class ObstacleDetectorNode(object):
	def __init__(self):
		self.node_name = "obstacle_detector_node"
		self.sub_image = rospy.Subscriber("~image_rect", Image, self.cbImage, queue_size=1)
		self.bridge = CvBridge()
		self.pub_visualize = rospy.Publisher("~tag_detections", AprilTagDetectionArray, queue_size=1)
		self.stop_sign_id = 10
		self.stop_sign_svm = fit_svm.SVM()

	def write_results(self, file, shape, size, ar, r, g, b, stop_sign = True):
		msg = "%d,%f,%f,%f,%f,%f,%f\n" % (stop_sign, shape, size, ar, r, g, b)
		self.write_to_file(file, msg)

	def write_to_file(self, file, msg):
		dir_path = os.path.dirname(os.path.realpath(__file__))
		file_loc = dir_path + "/" + file
		with open(file_loc, 'a') as f:
			f.write(msg)		

	def find_signs(self, img):
		shapes = find_stop_signs(img)

		published = False
		for shape in shapes:
			(x, y, w, h) = cv2.boundingRect(shape.approx)
			ar = w / float(h)
			if shape.size > 100 and ar >= 0.3: 
				in_vector = np.reshape(np.array([shape.shape, shape.size, ar, shape.r, shape.g, shape.b]), (1,6))
				is_stop_sign = self.stop_sign_svm.classify_point(in_vector)
				if is_stop_sign:
					self.mk_pub_msg(0, 10)
					published = True

		# send signal nothing was found
		if published == False:
			self.mk_pub_msg(100, -1)

	def mk_pub_msg(self, msg_z, msg_id):
		tag_detection_array = AprilTagDetectionArray()
		tag_pose = PoseStamped()
		tag_pose.pose.position.z = msg_z
		tag_detection = AprilTagDetection()
		tag_detection.pose = tag_pose
		tag_detection.id = msg_id
		tag_detection_array.detections.append(tag_detection)
		self.pub_visualize.publish(tag_detection_array)


	def cbImage(self, image):
		#self.find_signs(np.array(image.data))
		cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		np_image = np.array(cv_image)
		self.find_signs(np_image)



if __name__ == "__main__":
	rospy.init_node('obstacle_detector_node',anonymous=False)
	node = ObstacleDetectorNode()
	rospy.spin()