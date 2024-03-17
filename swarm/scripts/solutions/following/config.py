import cv2
import numpy as np

# Config for following solutions

FRAME_RATE = 30
FRAME_SIZE = (960, 720)
MARKER_SIZE = 50  # in pixels
MARKER_SIZE_RANGE = 15
MARKER_NUMBER = 304
X_RANGE = 125
Y_RANGE = 125
TIMEOUT = 10
STEP = 25
Z_STEP = 25
SPEED = 50
QUEUE_SIZE = 100
MARKER_LEN = 0.1
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
camera_distortion = np.array([0.1, 0.2, 0.0, 0.0, 0.0], dtype=np.float32)

DISTANCE = 125  # in cm
DISTANCE_RANGE = 25

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DET = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
