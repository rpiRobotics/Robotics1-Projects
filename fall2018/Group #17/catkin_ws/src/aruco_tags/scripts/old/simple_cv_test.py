import cv2
#img = cv2.imread('tiles_226.png')
#cv2.namedWindow('ImageWindow',cv2.WINDOW_NORMAL)
#cv2.imshow('ImageWindow',img)

#cv2.resizeWindow('ImageWindow', 600,600)
#cv2.waitKey(0)
#if cv2.waitKey(1) & 0xFF == ord('q'):

   # cv2.destroyAllWindows()

#cap = cv2.VideoCapture('test1.mp4')
cap = cv2.VideoCapture(0)
while(True):
    ret, frame = cap.read()
    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('frame', gray)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

