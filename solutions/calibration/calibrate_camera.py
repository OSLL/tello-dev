import cv2
import time
import numpy as np
import os
from drone.drone import Drone


class CalibrateCamera():

    GRID_SIZE = (5, 6) # Chess board size
    FRAMES_COUNT = 11
    OUTPUT_FOLDER = "calibration_results"

    def __init__(self):
        self.active = True
        self.saved_frames = []
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

    def stop(self):
        self.active = False

    def process_frame(self, frame):
        try:
            if self.calibrate(frame):
                if len(self.objpoints) == self.FRAMES_COUNT:
                    self.active = False
                else:
                    time.sleep(1)
        except Exception as exception:
            print("Exception in drone_camera_calibration:", exception)
        return frame

    def calibrate(self, frame):
        objp = np.zeros((self.GRID_SIZE[0] * self.GRID_SIZE[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.GRID_SIZE[0], 0:self.GRID_SIZE[1]].T.reshape(-1, 2)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_frame, self.GRID_SIZE, None)
        if ret == True:
            self.objpoints.append(objp)
            self.imgpoints.append(corners)
            self.saved_frames.append(frame)
            return True
        else:
            return False

    def write_resutls(self, mtx, dist):
        if not os.path.isdir(self.OUTPUT_FOLDER):
            os.mkdir(self.OUTPUT_FOLDER)
        with open(f"{self.OUTPUT_FOLDER}/camera_settings.npy", "wb") as f:
            np.save(f, mtx)
            np.save(f, dist)
        h,  w = self.saved_frames[0].shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        for i, frame in enumerate(self.saved_frames):
            cv2.imwrite(f"{self.OUTPUT_FOLDER}/frame{i}_origin.png", frame)
            cv2.imwrite(f"{self.OUTPUT_FOLDER}/frame{i}_undistort.png", cv2.undistort(frame, mtx, dist, None, newcameramtx))

    def drone_instructions(self, tello):
        last_cmd_time = time.process_time()
        while self.active:
            if time.process_time() - last_cmd_time > 10:
                tello.send_rc_control(0, 0, 0, 0)
                last_cmd_time = time.process_time()
            time.sleep(1)

    def main(self):
        self.is_stopped = False
        drone = Drone(self.drone_instructions, self.process_frame, show_stream=True)
        drone.run_program()
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            cv2.cvtColor(self.saved_frames[0], cv2.COLOR_BGR2GRAY).shape[::-1],
            None,
            None)
        if ret:
            self.write_resutls(mtx, dist)
            print("Calibration successfully completed")
        else:
            print("Calibration failed")
