#!/usr/bin/env python

"""
Author: Zac Ravichandran
Collection of functions for detecting potential stop signs
"""

import cv2
import imutils
import numpy as np

class FoundShape():
	def __init__(self, shape, contour, approx, cx, cy, ratio):
		self.shape = shape
		self.contour = contour
		self.approx = approx
		self.cx = cx
		self.cy = cy
		self.ratio = ratio
		self.size = self.get_size()
		self.r = 0
		self.g = 0
		self.b = 0

	def get_size(self):
		minx, maxx, miny, maxy = get_bounds(self.contour, self.ratio)
		return abs(minx - maxx) * abs(miny - maxy)

	def set_rgb(self, r, g, b):
		self.r = r 
		self.g = g 
		self.b = b


def detect(c):
	"""
	Use contours to identify shapes
	A contour list
	"""

	shape = "unidentified"
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.03* peri, True) # 0.04 * peri, True)

	if len(approx) == 3:
		shape = "triangle"
	elif len(approx) == 4:
		(x, y, w, h) = cv2.boundingRect(approx)
		ar = w / float(h)
		shape = "square" if abs(ar - 1) < 0.05 else "rect"
	elif len(approx) == 5:
		shape = "pent"
	else:
		shape = "circle";
	return len(approx), approx

def find_all_shapes(c_in, image, ratio, resized_img):
	found_shapes = []
	for c in c_in:
		M = cv2.moments(c)
		cX = int((M["m10"] / (M["m00"]+0.1)) * ratio)
		cY = int((M["m01"] / (M["m00"]+0.1)) * ratio)
		shape, approx = detect(c)
		found_shapes.append(FoundShape(shape, c, approx, cX, cY, ratio))

		# try getting color
		r, g ,b = get_rbg_values(resized_img, c)
		found_shapes[-1].set_rgb(np.mean(r), np.mean(g), np.mean(b))
	# show the output image
	return image, found_shapes

def get_bounds(contour, ratio):
	"""
	return xmin, xmax, ymin, ymax
	"""
	contour = contour.astype("float")
	contour *= ratio
	contour = contour.astype("int")
	return np.min(contour[:,:,0]), np.max(contour[:,:,0]), np.min(contour[:,:,1]), np.max(contour[:,:,1])

def canny_filter(img, low=100, high=200):
	edges = cv2.Canny(img,low, high)
	return edges


def find_blue(img):
	# blue sign is ~35178
	lower_blue = np.array([110,50,50])
	upper_blue = np.array([130,255,255])
	size = 30000

	return find_color(img, lower_blue, upper_blue, size)

def find_yellow(img):
	lower_yellow = np.array([20, 100, 100])
	upper_yellow = np.array([30, 255, 255])

	size = 300
	return find_color(img, lower_yellow, upper_yellow, size)

def find_white(img):
	lower_white = np.array([0,0,0])
	upper_white = np.array([0,0,255])
	size = 10000
	return find_color(img, lower_white, upper_white, size)

def find_red(img):
	#sizes 400, 2000, 40000
	lower_red = np.array([160, 100, 100])
	upper_red = np.array([179, 255, 255])
	size=10
	return find_color(img, lower_red, upper_red, size)

def find_color(img, lower_color, upper_color, size):
	# blue sign is ~35178
	resized = imutils.resize(img, width = 300)
	ratio = img.shape[0] / float(resized.shape[0])
	hsv_img = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv_img, lower_color, upper_color)
	return mask, ratio, size, resized

def get_rbg_values(img, contour):
	mask = img.copy()
	cv2.drawContours(mask, [contour], -1, (255,255,255), -1)
	#out = np.zeros_like(mask)
	b,g,r = cv2.split(img)
	bm, gm, rm = cv2.split(mask)
	return b[bm==255], g[gm==255], r[rm==255]

def find_stop_signs(img):
	preprocessd_img, ratio, size, resized_img = find_red(img)
	contours, hierarchy = cv2.findContours(preprocessd_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	img, found_shapes = find_all_shapes(contours, img, ratio, resized_img)

	return found_shapes

def find_ducks(img):
	preprocessd_img, ratio, size = find_yellow(img)
	contours, hierarchy = cv2.findContours(preprocessd_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	img, found_yellow_shapes = find_all_shapes(contours, img, ratio)

	return found_yellow_shapes