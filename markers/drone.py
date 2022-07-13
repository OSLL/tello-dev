import cv2
import time
import threading
from djitellopy import Tello


class Drone:

    def __init__(self, drone_prog, frame_processing, show_stream=True, frame_rate=None):
        self.tello = Tello()
        self.tello.connect()
        self.backreader = None
        self.active = False

        self.video_stream_thread = threading.Thread(target=self.frame_handler)
        print(f"Drone battery charge is {self.tello.get_battery()}%")

        self.drone_prog = drone_prog
        self.frame_processing = frame_processing
        self.show_stream = show_stream
        self.frame_rate = frame_rate

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
                if self.show_stream is True and show_frame is not None:
                    cv2.imshow("Drone stream", show_frame)
                    k = cv2.waitKey(1)
                    if k & 0xFF == ord("q"):
                        self.tello.land()
                        self.tello.end()
                        break
                time.sleep(1 / self.frame_rate)
        except Exception as exception:
            raise exception
            print("Exception in Drone.frame_handler:", exception)
