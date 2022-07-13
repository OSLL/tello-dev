import cv2
import time
import threading
from djitellopy import Tello
from utils.write_text import write_status_on_image


class Drone:

    def __init__(self, drone_prog, frame_processing, show_stream=True):
        self.tello = Tello()
        self.tello.connect()
        self.backreader = None
        self.active = False

        self.video_stream_thread = threading.Thread(target=self.frame_handler)

        self.drone_prog = drone_prog
        self.frame_processing = frame_processing
        self.show_stream = show_stream

    def run_program(self):
        self.active = True
        try:
            self.tello.streamon()
            # Using BackReader from djitellopy library to get frames
            self.backreader = self.tello.get_frame_read()
            self.video_stream_thread.start()
            time.sleep(2)
            self.drone_prog(self.tello)
        except Exception as exception:
            raise exception
            print("Exception in Drone.run_program:", exception)
        self.active = False
        if self.video_stream_thread is not None:
            self.video_stream_thread.join()
        self.tello.streamoff()
        self.tello.end()
        cv2.destroyAllWindows()

    def frame_handler(self):
        try:
            while self.active:

                if self.backreader is None:
                    break

                # Using BackReader from djitellopy library to get frames
                cur_frame = self.backreader.frame.copy()

                if cur_frame is None:
                    continue

                show_frame = self.frame_processing(cur_frame)
                write_status_on_image(show_frame, {"battery": f"{self.tello.get_battery()}%"})
                if self.show_stream is True and show_frame is not None:
                    cv2.imshow("Drone stream", show_frame)
                    cv2.waitKey(1)
        except Exception as exception:
            raise exception
            print("Exception in Drone.frame_handler:", exception)
