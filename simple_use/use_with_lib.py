import cv2
import time
import threading
from djitellopy import Tello

FRAME_RATE = 30
active = True


def drone_video(backreader):
    try:
        while active:
            # Using BackReader from djitellopy library to get frames
            cur_frame = backreader.frame.copy()

            # Using VideoCapture from opencv to get frames
            # res, cur_frame = backreader.read()

            if cur_frame is None:
                continue
            cv2.imshow("Drone stream", cur_frame)
            cv2.waitKey(1)
    except Exception as exception:
        print("Exception in drone_video:", exception)


def drone_prog(tello):
    tello.takeoff()
    tello.move_forward(100)
    time.sleep(1)
    tello.move_up(100)
    time.sleep(1)
    tello.rotate_clockwise(90)
    time.sleep(1)
    tello.move_down(50)
    time.sleep(1)
    tello.move_left(100)
    tello.flip("b")
    tello.flip("r")
    tello.flip("f")
    tello.land()


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

        video_stream_thread = threading.Thread(target=drone_video, args=(backreader, ))
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
