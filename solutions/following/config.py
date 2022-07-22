import cv2
import numpy as np

# Config for following solutions

FRAME_RATE = 30
FRAME_SIZE = (960, 720)
MARKER_SIZE = 50 # in pixels
MARKER_SIZE_RANGE = 15
MARKER_NUMBER = 304
X_RANGE = 125
Y_RANGE = 125
TIMEOUT = 10
STEP = 25
Z_STEP = 25
SPEED = 50
QUEUE_SIZE = 100

DISTANCE = 125 # in cm
DISTANCE_RANGE = 25

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
