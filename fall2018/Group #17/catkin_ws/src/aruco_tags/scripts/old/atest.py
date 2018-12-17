import inspect
from cv2 import aruco

for name, obj in inspect.getmembers(aruco):
    print obj
