import cv2
import numpy as np


MARKER_NUMBER = 304
MARKER_LEN = 6.5  # in cm
mtx = np.array([[918.293069, 0.000000, 497.076002],
                    [0.000000, 915.125633, 357.014392],
                    [0.000000, 0.000000, 1.000000]])
dist = np.array([-0.014463, 0.018835, 0.001203, 0.006318, 0.000000])

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
