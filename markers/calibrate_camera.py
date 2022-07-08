import cv2
import time
import threading
import numpy as np
import os
from djitellopy import Tello

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
GRID_SIZE = (5, 6) # Chess board size
FRAMES_COUNT = 11
OUTPUT_FOLDER = "calibration_results"

active = True
saved_frames = []


def write_resutls(mtx, dist):
    if not os.path.isdir(OUTPUT_FOLDER):
        os.mkdir(OUTPUT_FOLDER)
    print("Mtx is:", mtx)
    np.savetxt(f"{OUTPUT_FOLDER}/camera_matrix.txt", mtx, delimiter=", ")
    print("Dist is:", dist)
    np.savetxt(f"{OUTPUT_FOLDER}/camera_distortion.txt", dist, delimiter=", ")
    h,  w = saved_frames[0].shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    for i, frame in enumerate(saved_frames):
        cv2.imwrite(f"{OUTPUT_FOLDER}/frame{i}.png", cv2.undistort(frame, mtx, dist, None, newcameramtx))


def calibrate(frame, objpoints, imgpoints):
    objp = np.zeros((GRID_SIZE[0] * GRID_SIZE[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:GRID_SIZE[0], 0:GRID_SIZE[1]].T.reshape(-1, 2)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray_frame, GRID_SIZE, None)
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)
        saved_frames.append(frame)
        return True
    else:
        return False
    


def drone_camera_calibration(backreader):
    global active
    try:
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        while active:
            # Using BackReader from djitellopy library to get frames
            cur_frame = backreader.frame.copy()

            # Using VideoCapture from opencv to get frames
            # res, cur_frame = backreader.read()

            if cur_frame is None:
                continue

            cv2.imshow("Tello drone", cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY))
            cv2.waitKey(1)

            if calibrate(cur_frame, objpoints, imgpoints):
                print("Find chess")
                if len(objpoints) == FRAMES_COUNT:
                    active = False
                else:
                    time.sleep(1.5)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, cv2.cvtColor(saved_frames[0], cv2.COLOR_BGR2GRAY).shape[::-1], None, None)
        if ret:
            write_resutls(mtx, dist)
        else:
            print("Calibration failed")
    except Exception as exception:
        print("Exception in drone_camera_calibration:", exception)


def drone_prog(tello):
    while active:
        time.sleep(1)


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    video_stream_thread = None
    try:
        print(f"Battery charge is {tello.get_battery()}%")
        tello.streamon()
        time.sleep(2)
        # Using BackReader from djitellopy library to get frames
        backreader = tello.get_frame_read()

        # Using VideoCapture from opencv to get frames
        # vidcap = cv2.VideoCapture("udp://192.168.10.1:11111", cv2.CAP_FFMPEG)

        video_stream_thread = threading.Thread(target=drone_camera_calibration, args=(backreader, ))
        video_stream_thread.start()
        drone_prog(tello)
    except Exception as exception:
        print("Exception:", exception)
    active = False
    if video_stream_thread is not None:
        video_stream_thread.join()
    # For VideoCapture from opencv
    # vidcap.release()
    tello.end()
    cv2.destroyAllWindows()
