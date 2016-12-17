import cv2

from numpy import array, all, zeros, ones, rot90
from matplotlib.mlab import find

from .hamming_marker import HammingMarker, marker_size
from .hamming import decode




def detect_markers(img, marker_ids=None):
    width, height, _ = img.shape

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 10, 100)
    
    # OpenCV Version 3.1.0 Version
    _, contours, _ = cv2.findContours(edges.copy(),
                                      cv2.RETR_TREE,
                                      cv2.CHAIN_APPROX_NONE)

    # OpenCV - Latest Stable Version (2.x)
    #contours, _ = cv2.findContours(edges.copy(),
    #                                  cv2.RETR_TREE,
    #                                  cv2.CHAIN_APPROX_NONE)

    # We only keep the big enough contours
    min_area = width * height * .01
    contours = [c for c in contours if cv2.contourArea(c) > min_area]

    warped_size = 9 * 10
    canonical_marker_coords = array(((0, 0),
                                     (warped_size - 1, 0),
                                     (warped_size - 1, warped_size - 1),
                                     (0, warped_size - 1)),
                                    dtype='float32')

    markers = []

    for c in contours:
        approx_curve = cv2.approxPolyDP(c, len(c) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        sorted_curve = array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve,
                                                   canonical_marker_coords)

        warped_img = cv2.warpPerspective(img, persp_transf,
                                         (warped_size, warped_size))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, warped_bin = cv2.threshold(warped_gray, 50, 255, cv2.THRESH_BINARY)

        marker = warped_bin.reshape([marker_size,
                                     warped_size / marker_size,
                                     marker_size,
                                     warped_size / marker_size])
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        # Eliminate the entirely black or entirely white markers
        # for robustness purposes
        sub_marker = marker[1:-1, 1:-1]
        sub_size = marker_size - 2
        if (all(sub_marker == zeros((sub_size, sub_size))) or
                all(sub_marker == ones((sub_size, sub_size)))):
            continue

        for _ in range(4):
            try:
                code = decode(sub_marker).flatten()[::-1]
                id = (2 ** find(code == 1)).sum()
                markers.append(HammingMarker(id=id, contours=approx_curve, img_size=(width, height)))
            except ValueError:  # The hamming code is incorrect
                pass

            sub_marker = rot90(sub_marker)

    # Remove duplicates
    markers = {m.id: m for m in markers}.values()

    return markers
