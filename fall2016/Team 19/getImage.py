import cv2

if __name__ == "__main__":
	camera = cv2.VideoCapture(1)

	nope, capture = camera.read()

	cv2.imwrite("Picture 1.jpg", capture)

	del(camera)