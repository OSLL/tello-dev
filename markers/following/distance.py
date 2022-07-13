import cv2
import time
import math
import numpy as np
from drone import Drone
from enum import Enum, auto


FRAME_RATE = 30
FRAME_SIZE = (960, 720)
# MARKER_LEN = 6.5 # Size in cm
MARKER_SIZE = 50 # in pixels
MARKER_SIZE_RANGE = 15
MARKER_NUMBER = 304
X_RANGE = 125
Y_RANGE = 125
TIMEOUT = 10
STEP = 25
Z_STEP = 25
SPEED = 100


video_writer = cv2.VideoWriter('drone_stream.avi', cv2.VideoWriter_fourcc(*'XVID'), FRAME_RATE, FRAME_SIZE)
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

camera_matrix = np.array([
    [1.015503547798015916e+03, 0.000000000000000000e+00, 4.823323956404478849e+02],
    [0.000000000000000000e+00, 1.000515091195654691e+03, 2.890797713914782321e+02],
    [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
])

camera_distortion = np.array([-7.609968082285716029e-02, 5.002166244275487728e-01, 1.483359646311869107e-03, 7.477616561322539095e-03, -1.048118231094581576e+00])


class State(Enum):
    LEFT = auto()
    RIGHT = auto()
    BACK = auto()
    FORWARD = auto()
    LEFT_FORWARD = auto()
    RIGHT_FORWARD = auto()
    LEFT_BACK = auto()
    RIGHT_BACK = auto()
    HOVER = auto()
    END = auto()
    EMPTY = auto()
    START = auto()
    MOVE = auto()


actions = []
last_state = State.START
last_time = time.process_time()


def stop():
    actions.clear()
    actions.insert(0, (State.END, np.zeros(3, dtype=np.int32)))


def dist_beetwen_points(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_state(center, marker_size, m_point):
    x_diff = m_point[0] - center[0] # diff on frame
    y_diff = m_point[1] - center[1] # diff on frame
    x = y = z = 0
    if x_diff > X_RANGE:
        y = -STEP
    elif x_diff < -X_RANGE:
        y = STEP
    if y_diff > Y_RANGE:
        z = -Z_STEP
    elif y_diff < -Y_RANGE:
        z = Z_STEP
    if marker_size < MARKER_SIZE - MARKER_SIZE_RANGE:
        x = STEP
    elif marker_size > MARKER_SIZE + MARKER_SIZE_RANGE:
        x = -STEP
    if x == 0 and y == 0 and z == 0:
        return (State.HOVER, np.zeros(3, dtype=np.int32))
    return (State.MOVE, np.array([x, y, z], dtype=np.int32))


def process_frame(frame):
    global last_state, last_time
    center = (frame.shape[1] // 2, frame.shape[0] // 2)
    '''if last_state in [State.START, State.END]:
        cv2.putText(frame, f"Last state is {last_state}", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        frame = cv2.rectangle(frame, (center[0] - X_RANGE, center[1] - Y_RANGE), (center[0] + X_RANGE, center[1] + Y_RANGE), (0, 255, 0), 2)
        video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame
    ids = None
    is_find = True
    if last_state == State.EMPTY:'''
    is_find = True
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
    if ids is not None:
        for i in range(len(ids)):
            if ids[i] == MARKER_NUMBER:
                m_id = ids[i]
                m_corner = corners[i]
                break
        else:
            is_find = False
        if is_find:
            frame = cv2.aruco.drawDetectedMarkers(frame, np.array([m_corner]), np.array([m_id]))
            rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), 6.5, camera_matrix, camera_distortion)
            # tvecs[0][0] is [smth, smth, distance]
            tvec = tvecs[0][0]
            rvec = rvecs[0][0]
            i_str = f"tvec: {tvecs.tolist()}"
            cv2.putText(frame, i_str, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if ids is None or not is_find:
        cv2.putText(frame, f"No marker", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    '''cv2.putText(frame, f"Last state is {last_state}", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    frame = cv2.rectangle(frame, (center[0] - X_RANGE, center[1] - Y_RANGE), (center[0] + X_RANGE, center[1] + Y_RANGE), (0, 255, 0), 2)
    video_writer.write(cv2.resize(frame, FRAME_SIZE))'''
    return frame


def drone_instructions(tello):
    time.sleep(20)


def main():
    drone = Drone(drone_instructions, process_frame, show_stream=True, frame_rate=FRAME_RATE)
    drone.run_program()
    video_writer.release()