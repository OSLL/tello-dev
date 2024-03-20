import time
from queue import Queue, Empty
from drone.drone import Drone
from swarm.scripts.solutions.following.config import *
from swarm.scripts.solutions.following.config import camera_matrix, camera_distortion, MARKER_LEN
from solutions.following.state import State
from utils.write_text import write_status_on_image


class FollowMarkerWithCoords:

    def __init__(self, initial_position=()):
        self.video_writer = cv2.VideoWriter('avi/drone_stream.avi', cv2.VideoWriter_fourcc(*'XVID'), FRAME_RATE, FRAME_SIZE)
        self.actions = Queue(QUEUE_SIZE)
        self.last_state = State.START
        self.last_time = time.process_time()
        self.last_battery_charge = 0
        self.is_stopped = False
        self.initial_position = initial_position

    def stop(self):
        self.is_stopped = True

    def get_state(self, tvec):
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
                    print("Marker not found. Landing...")
                    self.actions.put((State.END, None))
                    self.last_state = State.END
                    is_find = False
        if is_find:
            write_status_on_image(frame, {"Battery": f"{self.last_battery_charge}%", "Last state": self.last_state})
            self.video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame

    def drone_instructions(self, tello):
        tello.takeoff()
        tello.move_xyz_speed(*self.initial_position, SPEED)  # начальная позиция
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