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
        # self.drone_id = None
        # self.get_drone_id()
        self.drone_id = 1
        self.coord_drone = None
        self.num_of_rotate = 0

    def stop(self):
        self.is_stopped = True




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
        rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), MARKER_LEN, camera_matrix, camera_distortion)
        state = self.get_state(tvecs[0][0])
        if state == State.MOVE:
            self.actions.put(state)
            self.last_state = state

    def marker_coords(self, m_corner, marker_id):
        self.last_time = time.process_time()
        _ , tvecs, _  = cv2.aruco.estimatePoseSingleMarkers(np.array([m_corner]), MARKER_LEN, camera_matrix,
                                                              camera_distortion)
        tvec = tvecs[0][0]
        marker_x, marker_y, marker_z = tvec[2], tvec[0], tvec[1]
        x = z = y = 0
        drone_x = x + marker_x
        drone_y = y + marker_y
        drone_z = z + marker_z
        print(drone_x,drone_y,drone_x)
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
            x = y = z = 0 # may be z = 50
            x = x + drone_x_zero
            y = y + drone_y_zero
            z = z + drone_z_zero
            self.coord_drone = (x,y,z)
            print(f"{self.drone_id}, {x}, {y}, {z}\n")
            self.save_drone_info(f"{self.drone_id}, {x}, {y}, {z}\n")
        elif marker_id == MARKER_MOVE:
            x = y = z = 0 # may be z = 50
            x = x + drone_x_zero + self.coord_drone[0]
            y = y + drone_y_zero + self.coord_drone[1]
            z = z + drone_z_zero + self.coord_drone[2]
            self.save_drone_info(f"MARKER_MOVE, {x}, {y}, {z}\n")
        return 0

    def save_drone_info(self, string):
        try:
            with open(".\drone_flight_logs.txt", 'a') as file:
                file.write(string)
            print(f"Координаты успешно сохранены в файл drone_flight_logs.txt")
            return True
        except Exception as e:
            print(f"Ошибка при сохранении координат: {e}")
            return False

    def process_frame(self, frame):
        ids = None
        is_find = True
        if self.last_state == State.READY:
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
                    self.flag = 1
                    print("START LINE COORDS")
                    print("degree rotate - ",self.num_of_rotates)
                    # i=1
                    # for _ in range(self.num_of_rotates, 360, 45):
                    self.num_of_rotate = 360 - self.num_of_rotates
                    self.actions.put((State.ROTATE, None))
                    #     print(i)
                    #     i+=1

                    # self.num_of_rotates = 0
                    print("END LINE COORDS")

                if self.flag == 1:
                    if marker_id == MARKER_MOVE:
                        self.marker_coords(marker_corner, MARKER_MOVE)
                        state = self.move_to_marker()
                        self.actions.put(state)
                        self.last_state = state

                    if marker_id == MARKER_STOP:
                        print("Send END")
                        self.actions.put((State.END, None))
                        self.last_state = State.END
                    if marker_id == MARKER_CENTER:
                        print("Send ROTATE 2")
                        self.actions.put((State.ROTATE, None))
                        self.last_state = State.ROTATE
                else:
                    print("Send ROTATE 2")
                    self.actions.put((State.ROTATE, None))
                    self.last_state = State.ROTATE
            else:
                # if not self.flag:
                print("Send ROTATE 1")
                self.actions.put((State.ROTATE, None))
                self.last_state = State.ROTATE
                if time.process_time() - self.last_time > TIMEOUT:
                    print("Send END2")
                    self.actions.put((State.END, None))
                    self.last_state = State.END
            # time.sleep(1.5)
        write_status_on_image(frame, {"Battery": f"{self.last_battery_charge}%", "Last state": self.last_state})
        self.video_writer.write(cv2.resize(frame, FRAME_SIZE))
        return frame

    def drone_instructions(self, tello):
        tello.takeoff()
        # tello.move_up(50)
        self.last_time = time.process_time()
        self.last_state = State.READY
        # x = z = y = tello.get_current_xyz()

        while not self.is_stopped:
            try:
                state, movements = self.actions.get(block=False, timeout=0.25)
                self.actions.task_done()
                if state == State.END:
                    break
                elif state == State.MOVE:
                    # print("DO MOVE")
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

    def main(self):
        self.is_stopped = False
        drone = Drone(self.drone_instructions, self.process_frame, show_stream=True)
        drone.run_program()
        self.video_writer.release()