import docker
import os
import cv2
import time
import math
import numpy as np
from queue import Queue, Empty
from drone.drone import Drone
from swarm.scripts.solutions.following.config import FRAME_RATE, FRAME_SIZE, DISTANCE, DISTANCE_RANGE,\
    ARUCO_DICT, ARUCO_PARAMS, QUEUE_SIZE, TIMEOUT, MARKER_CENTER, ROTATE_DEGREE, MARKER_MOVE, MARKER_STOP, SPEED
from swarm.scripts.solutions.following.config import camera_matrix, camera_distortion, MARKER_LEN
from swarm.scripts.solutions.following.state import State
from utils.write_text import write_status_on_image


from jsonrpcclient import request
import requests

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
        self.num_of_rotates = 0
        self.flag = 0
        self.flag_two = 0
        self.drone_id = 1
        self.coord_drone = None
        self.num_of_rotate = 0

        self.swarm_coordinated = False
        self.coord_drone = None

        self.rotate_flag = 1
        self.move_flag = 0
        self.move_marker = 0

    def stop(self):
        self.is_stopped = True


    def move_to_marker(self):
        x, y, z =self.move_marker
        x_un = self.coord_drone[0] - x
        y_un = self.coord_drone[1] - y
        z_un = self.coord_drone[2] - z
        self.coord_drone = (x_un, y_un, z_un)
        coord_drone = (x_un, y_un, z_un)
        angle_degrees = 360 - self.num_of_rotates

        # Получаем матрицу поворота из вектора ориентации
        angle_radians = math.radians(angle_degrees)

        # Преобразуем угол поворота в матрицу поворота
        R = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                      [np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])

        # Исходные координаты дрона
        drone_coords = np.array(coord_drone)

        # Применяем поворот к координатам дрона
        rotated_coords = np.dot(R, drone_coords)

        # Добавляем координаты нулевой позиции дрона
        x = int(rotated_coords[0] - 10)
        y = int(rotated_coords[1])
        z = int(rotated_coords[2])

        return (State.MOVE, np.array([x, y, z], dtype=np.int32))

    def find_marker(self, frame):
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == MARKER_CENTER:
                    return (ids[i], corners[i])
                if ids[i] == MARKER_STOP:
                    return (ids[i], corners[i])
                if ids[i] == MARKER_MOVE:
                    return (ids[i], corners[i])
        return None

    def make_action(self, m_corner):
        self.last_time = time.process_time()
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), MARKER_LEN, camera_matrix,
                                                              camera_distortion)
        state = self.get_state(tvecs[0][0])
        if state == State.MOVE:
            self.actions.put((state, None))
            self.last_state = state

    def marker_coords(self, m_corner, marker_id):
        self.last_time = time.process_time()
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), MARKER_LEN, camera_matrix,
                                                          camera_distortion)
        tvec = tvecs[0][0]
        marker_x, marker_y, marker_z = tvec[2], tvec[0], tvec[1]
        x = z = y = 0
        drone_x = x + marker_x
        drone_y = y + marker_y
        drone_z = z + marker_z
        print(drone_x, drone_y, drone_x)
        angle_radians = math.radians(self.num_of_rotates)

        # Преобразуем угол поворота в матрицу поворота
        R = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                      [np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])

        # Исходные координаты дрона
        drone_coords = np.array([drone_x, drone_y, drone_z])

        # Применяем поворот к координатам дрона
        rotated_coords = np.dot(R, drone_coords)

        # Добавляем координаты нулевой позиции дрона
        drone_x_zero = int(rotated_coords[0])
        drone_y_zero = int(rotated_coords[1])
        drone_z_zero = int(rotated_coords[2])

        # Добавляем смещение для учета положения дрона относительно маркера
        if marker_id == MARKER_CENTER:
            x = y = z = 0  # may be z = 50
            x = x + drone_x_zero
            y = y + drone_y_zero
            z = z + drone_z_zero
            self.coord_drone = (x, y, z)
            print(f"{self.drone_id}, {x}, {y}, {z}\n")
        elif marker_id == MARKER_MOVE:
            x = y = z = 0  # may be z = 50
            x = x + drone_x_zero + self.coord_drone[0]
            y = y + drone_y_zero + self.coord_drone[1]
            z = z + drone_z_zero + self.coord_drone[2]
            self.move_marker = (x,y,z)
        return (x, y, z)

    def process_frame(self, frame):
        ids = None
        is_find = True
        if self.last_state == State.READY and self.flag == int(self.swarm_coordinated):
            print("State.READY")
            ret = self.find_marker(frame)
            if ret is not None:
                print("МАРКЕР НАЙДЕН")
                marker_id, marker_corner = ret
                print(marker_id)
                frame = cv2.aruco.drawDetectedMarkers(frame, np.array([marker_corner]), np.array([marker_id]))
                if marker_id == MARKER_CENTER and not self.flag:
                    print("Send COORDS")
                    self.marker_coords(marker_corner, MARKER_CENTER)
                    print("START LINE COORDS")
                    print("degree rotate - ",self.num_of_rotates)

                    # vvvvvvvvvvvv ОПРЕДЕЛИТЬ КОРРЕКТНЫЙ ИРЛ!!! vvvvvvvvvvvvvvvv
                    url = f"http://{""}:65001"
                    params = {
                        "command": "turn_starting_position",
                        "coordinates": self.coord_drone
                    }
                    req = request("exec", params)
                    response = requests.post(url, json=req)
                    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    print("END LINE COORDS")
                if self.flag == 1 and self.rotate_flag == 0:
                    if marker_id == MARKER_MOVE:
                        coord = self.marker_coords(marker_corner, MARKER_MOVE)
                        # print(мы нашили маркер мув и угол)
                        url = f"http://{""}:65001"
                        params = {
                            "command": "save_coord_move_marker",
                            "coordinates": coord
                        }
                        req = request("exec", params)
                        response = requests.post(url, json=req)
                if self.flag == 1:
                    if self.move_flag == 1:
                        state = self.move_to_marker()
                        self.actions.put(state)
                        self.last_state = state
                        self.move_flag = 0
                    if marker_id == MARKER_STOP:
                        print("Send END")
                        # vvvvvvvvvvvv ОПРЕДЕЛИТЬ КОРРЕКТНЫЙ ИРЛ!!! vvvvvvvvvvvvvvvv
                        url = f"http://{""}:65001"
                        params = {
                            "command": "land"
                        }
                        req = request("exec", params)
                        response = requests.post(url, json=req)
                        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                    if marker_id == MARKER_CENTER:
                        if self.rotate_flag == 1:
                            print("Send ROTATE 2")
                            self.actions.put((State.ROTATE, None))
                            self.last_state = State.ROTATE
                            self.rotate_flag = 0
                        else:
                            url = f"http://{""}:65001"
                            params = {
                                "command": "set_rotate"
                            }
                            req = request("exec", params)
                            response = requests.post(url, json=req)
                else:
                    if self.rotateflag == 1:
                        print("Send ROTATE 2")
                        self.actions.put((State.ROTATE, None))
                        self.last_state = State.ROTATE
                        if self.flag == 1:
                            self.rotate_flag = 0

            else:
                print("Send ROTATE 1")
                if self.rotate_flag == 0:
                    url = f"http://{""}:65001"
                    params = {
                        "command": "set_rotate"
                    }
                    req = request("exec", params)
                    response = requests.post(url, json=req)
                if self.rotate_flag == 1:
                    self.actions.put((State.ROTATE, None))
                    self.last_state = State.ROTATE
                    if self.flag == 1:
                       self.rotate_flag = 0 
                if time.process_time() - self.last_time > TIMEOUT:
                    print("Send END2")
                    url = f"http://{""}:65001"
                    params = {
                        "command": "land",
                    }
                    req = request("exec", params)
                    response = requests.post(url, json=req)

        write_status_on_image(frame, {"Battery": f"{self.last_battery_charge}%", "Last state": self.last_state})
        self.video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame

    def drone_instructions(self, tello):
        tello.takeoff()
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
                    print(movements)
                    tello.go_xyz_speed(int(x), int(y), int(z), SPEED)
                elif state == State.ROTATE:
                    if self.num_of_rotate == 0:

                        if self.num_of_rotates < 360:
                            x = ROTATE_DEGREE
                            tello.rotate_clockwise(x)
                            self.num_of_rotates += x
                            print("Rotate Degree", self.num_of_rotates)

                            self.last_time -= 5
                    else:
                        x = self.num_of_rotate
                        tello.rotate_clockwise(x)
                        self.num_of_rotate = 0
                        print("Rotate on first pose", self.num_of_rotate)
                        self.num_of_rotates = 0
            except Empty:
                self.last_state = State.READY
            self.last_battery_charge = tello.get_battery()
        tello.land()

    def turn_starting_position(self, coord):
        if not self.flag and self.coord_drone == coord:
            self.flag = 1
            self.num_of_rotate = 360 - self.num_of_rotates
            self.actions.put((State.ROTATE, None))
            self.last_state = State.ROTATE
            self.rotate_flag = 0    
        return "OK"

    
    def land(self):
        self.actions.put((State.END, None))
        self.last_state = State.END
        return "OK"

    def set_swarm_is_coordinated(self):
        self.swarm_coordinated = True
        return "OK"
    
    def set_rotate_flag(self):
        self.rotate_flag = 1
        return "OK"

    def save_move_marker(self, coord):
        self.move_flag = 1
        self.move_marker = coord
        return "OK"
    
    def save_coord_all_drones(self, coord_all_drones):
        self.coord_all_drones = coord_all_drones
        return "OK"
        

    def main(self):
        self.is_stopped = False
        drone = Drone(self.drone_instructions, self.process_frame, show_stream=True)
        drone.run_program()
        self.video_writer.release()