import cv2
import time
import math
import numpy as np
from queue import Queue, Empty
from drone.drone import Drone
from solutions.following.config import FRAME_RATE, FRAME_SIZE, DISTANCE, DISTANCE_RANGE,\
    ARUCO_DICT, ARUCO_PARAMS, QUEUE_SIZE, TIMEOUT, MARKER_NUMBER, SPEED
from config import camera_matrix, camera_distortion, MARKER_LEN
from solutions.following.state import State
from utils.write_text import write_status_on_image


def dist_beetwen_points(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


class FollowMarkerWithCoords():

    def __init__(self):
        self.video_writer = cv2.VideoWriter('drone_stream.avi', cv2.VideoWriter_fourcc(*'XVID'), FRAME_RATE, FRAME_SIZE)
        self.actions = Queue(QUEUE_SIZE)
        self.last_state = State.START
        self.last_time = time.process_time()
        self.last_battery_charge = 0
        self.is_stopped = False

    def stop(self):
        self.is_stopped = True

    def get_state(self, tvec):
        # marker:
        # x -- left(-) right(+)
        # y -- up(-) down(+)
        # z -- forward(-) back(+)
        #
        # drone:
        # x -- forward(+) back(-)
        # y -- left(+) right(-)
        # z -- up(+) down(-)
        marker_x, marker_y, marker_z = tvec[2], tvec[0], tvec[1]
        x = y = z = 0
        if abs(marker_y) > 20:
            y = -int(marker_y)
        if abs(marker_z) > 20:
            z = -int(marker_z)
        if abs(DISTANCE - marker_x) > DISTANCE_RANGE:
            x = int(marker_x - DISTANCE)
        if x == 0 and y == 0 and z == 0:
            return None
        return (State.MOVE, np.array([x, y, z], dtype=np.int32))

    def find_marker(self, frame):
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == MARKER_NUMBER:
                    return (ids[i], corners[i])
        return None

    def move_to_marker(self, m_corner):
        self.last_time = time.process_time()
        _ , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), MARKER_LEN, camera_matrix, camera_distortion)
        state = self.get_state(tvecs[0][0])
        if state is not None:
            self.actions.put(state)
            self.last_state = state

    def process_frame(self, frame):
        ids = None
        is_find = True
        if self.last_state == State.READY:
            ret = self.find_marker(frame)   
            if ret is not None:
                marker_id, marker_corner = ret
                frame = cv2.aruco.drawDetectedMarkers(frame, np.array([marker_corner]), np.array([marker_id]))
                self.move_to_marker(marker_corner)
            else:
                if time.process_time() - self.last_time > TIMEOUT:
                    print("Send END")
                    self.actions.put((State.END, None))
                    self.last_state = State.END
        write_status_on_image(frame, {"Battery": f"{self.last_battery_charge}%", "Last state": self.last_state})
        self.video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame

    def drone_instructions(self, tello):
        tello.takeoff()
        tello.move_up(50)
        self.last_time = time.process_time()
        self.last_state = State.READY
        while not self.is_stopped:
            try:
                state, movements = self.actions.get(block=False, timeout=0.25)
                self.actions.task_done()
                if state == State.END:
                    break
                elif state == State.MOVE:
                    x, y, z = movements
                    tello.go_xyz_speed(int(x), int(y), int(z), SPEED)
            except Empty:
                self.last_state = State.READY
            self.last_battery_charge = tello.get_battery()
        tello.land()

    def main(self):
        self.is_stopped = False
        drone = Drone(self.drone_instructions, self.process_frame, show_stream=True)
        drone.run_program()
        self.video_writer.release()