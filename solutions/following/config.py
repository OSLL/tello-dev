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

# Camera settings
# TODO: do a read from a file

camera_matrix = np.array([
    [1.015503547798015916e+03, 0.000000000000000000e+00, 4.823323956404478849e+02],
    [0.000000000000000000e+00, 1.000515091195654691e+03, 2.890797713914782321e+02],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
])

camera_distortion = np.array([-7.609968082285716029e-02, 5.002166244275487728e-01, 1.483359646311869107e-03, 7.477616561322539095e-03, -1.048118231094581576e+00])