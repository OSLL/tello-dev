import cv2
import time
import math
import numpy as np
from drone import Drone
from enum import Enum, auto


FRAME_RATE = 30
FRAME_SIZE = (640, 480)
# MARKER_LEN = 6.5 # Size in cm
MARKER_SIZE = 50 # in pixels
MARKER_SIZE_RANGE = 15
MARKER_NUMBER = 304
X_RANGE = 125
Y_RANGE = 125
TIMEOUT = 10
STEP = 25
SPEED = 50


video_writer = cv2.VideoWriter('drone_stream.avi', cv2.VideoWriter_fourcc(*'XVID'), FRAME_RATE, FRAME_SIZE)
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

# camera_matrix = np.array([
#     [1.015503547798015916e+03, 0.000000000000000000e+00, 4.823323956404478849e+02],
#     [0.000000000000000000e+00, 1.000515091195654691e+03, 2.890797713914782321e+02],
#     [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
# ])

# camera_distortion = np.array([-7.609968082285716029e-02, 5.002166244275487728e-01, 1.483359646311869107e-03, 7.477616561322539095e-03, -1.048118231094581576e+00])


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


actions = []
last_state = State.START
last_time = time.process_time()


def stop():
    actions.clear()
    # actions.insert(0, (State.END, np.zeros(3, dtype=np.int32)))
    actions.insert(0, State.END)


def dist_beetwen_points(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def get_state(frame, marker_size, m_point):
    center = (frame.shape[1] // 2, frame.shape[0] // 2)
    x_diff = m_point[0] - center[0]
    y_diff = m_point[1] - center[1]
    if x_diff > X_RANGE and y_diff > Y_RANGE:
        return State.RIGHT_BACK
    elif x_diff > X_RANGE and y_diff < -Y_RANGE:
        return State.RIGHT_FORWARD
    elif x_diff < -X_RANGE and y_diff > Y_RANGE:
        return State.LEFT_BACK
    elif x_diff < -X_RANGE and y_diff < -Y_RANGE:
        return State.LEFT_FORWARD
    elif x_diff > X_RANGE:
        return State.RIGHT
    elif x_diff < -X_RANGE:
        return State.LEFT
    elif y_diff > Y_RANGE:
        return State.BACK
    elif y_diff < -Y_RANGE:
        return State.FORWARD
    elif marker_size < MARKER_SIZE - MARKER_SIZE_RANGE:
        return State.FORWARD
    elif marker_size > MARKER_SIZE + MARKER_SIZE_RANGE:
        return State.BACK
    return State.HOVER


def process_frame(frame):
    global last_state, last_time
    center = (frame.shape[1] // 2, frame.shape[0] // 2)
    if last_state in [State.START, State.END]:
        cv2.putText(frame, f"Last state is {last_state}", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame = cv2.rectangle(frame, (center[0] - X_RANGE, center[1] - Y_RANGE), (center[0] + X_RANGE, center[1] + Y_RANGE), (0, 255, 0), 2)
        video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame
    ids = None
    is_find = True
    if last_state == State.EMPTY:
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
                last_time = time.process_time()
                frame = cv2.aruco.drawDetectedMarkers(frame, np.array([m_corner]), np.array([m_id]))
                p1, p2, p3, p4 = m_corner[0]
                marker_size = max(
                    dist_beetwen_points(p1, p2),
                    dist_beetwen_points(p2, p3),
                    dist_beetwen_points(p3, p4),
                    dist_beetwen_points(p4, p1)
                )
                state = get_state(frame, marker_size, p1)
                if last_state != state:
                    actions.append(state)
                    last_state = state
        if ids is None or not is_find:
            if time.process_time() - last_time > TIMEOUT:
                print("Send END")
                actions.append(State.END)
                last_state = State.END
            elif last_state != State.HOVER:
                actions.append(State.HOVER)
                last_state = State.HOVER
    # print(time.process_time() - last_time)
    cv2.putText(frame, f"Last state is {last_state}", (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    frame = cv2.rectangle(frame, (center[0] - X_RANGE, center[1] - Y_RANGE), (center[0] + X_RANGE, center[1] + Y_RANGE), (0, 255, 0), 2)
    video_writer.write(cv2.resize(frame, FRAME_SIZE))
    return frame


def drone_instructions(tello):
    global last_state, last_time
    tello.takeoff()
    tello.move_up(150)
    # tello.go_xyz_speed(0, 0, 50, 50)
    # tello.send_command_with_return("")
    last_exec_time = time.process_time()
    last_time = last_exec_time
    last_state = State.EMPTY
    while True:
        if len(actions) == 0:
            if time.process_time() - last_exec_time > 1:
                last_state = State.EMPTY
            continue
        last_exec_time = time.process_time()
        state = actions.pop(0)
        if state == State.END:
            break
        elif state == State.HOVER:
            pass
        else:
            z = 0
            x = 0
            y = 0
            if state == State.FORWARD:
                x = STEP
            elif state == State.BACK:
                x = -STEP
            elif state == State.LEFT:
                y = STEP
            elif state == State.RIGHT:
                y = -STEP
            elif state == State.LEFT_FORWARD:
                x = STEP
                y = STEP
            elif state == State.RIGHT_FORWARD:
                x = STEP
                y = -STEP
            elif state == State.LEFT_BACK:
                x = -STEP
                y = STEP
            elif state == State.RIGHT_BACK:
                x = -STEP
                y = -STEP
            tello.go_xyz_speed(x, y, z, SPEED)
    tello.land()


def main():
    drone = Drone(drone_instructions, process_frame, show_stream=True, frame_rate=FRAME_RATE)
    drone.run_program()
    video_writer.release()