import cv2
import numpy as np
from tello_ros.utils import read_camera_settings


MARKER_NUMBER = 304
MARKER_LEN = 6.5  # in cm
mtx, dist = read_camera_settings("camera.yaml")

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
