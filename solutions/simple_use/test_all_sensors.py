import cv2
import time
import threading
from djitellopy import Tello
from utils.write_text import write_status_on_image
from utils.drone_status import get_full_drone_status


class TestAllSensors():

    def __init__(self):
        self.active = False
        self.video_stream_thread = None
        self.tello = Tello()

    def drone_video(self, backreader):
        try:
            while self.active:
                # Using BackReader from djitellopy library to get frames
                cur_frame = backreader.frame.copy()

                # Using VideoCapture from opencv to get frames
                # res, cur_frame = backreader.read()

                if cur_frame is None:
                    continue
                write_status_on_image(cur_frame, get_full_drone_status(self.tello))
                cv2.imshow("Drone stream", cur_frame)
                cv2.waitKey(1)
        except Exception as exception:
            print("Exception in drone_video:", exception)

    def drone_prog(self):
        self.tello.takeoff()
        self.tello.move_forward(100)
        time.sleep(0.3)
        self.tello.move_up(100)
        time.sleep(0.3)
        self.tello.rotate_clockwise(90)
        time.sleep(0.3)
        self.tello.move_down(50)
        time.sleep(0.3)
        self.tello.move_left(100)
        self.tello.flip("b")
        self.tello.land()


    def main(self):
        self.tello.connect()
        self.video_stream_thread = None
        self.active = True
        try:
            self.tello.streamon()
            # Wait for stream activation
            time.sleep(2)
            # Using BackReader from djitellopy library to get frames
            backreader = self.tello.get_frame_read()

            # Using VideoCapture from opencv to get frames
            # vidcap = cv2.VideoCapture("udp://192.168.10.1:11111", cv2.CAP_FFMPEG)

            self.video_stream_thread = threading.Thread(target=self.drone_video, args=(backreader, ))
            self.video_stream_thread.start()
            self.drone_prog()
        except Exception as exception:
            print("Exception in main:", exception)
        self.active = False
        if self.video_stream_thread is not None:
            self.video_stream_thread.join()
        # For VideoCapture from opencv
        # vidcap.release()
        self.tello.end()
        cv2.destroyAllWindows()

    def stop(self):
        self.active = False
        self.tello.end()
