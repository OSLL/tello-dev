import cv2
import numpy as np
from drone.drone import Drone
from utils.write_text import write_status_on_image
from utils.drone_status import get_full_drone_status


class DefaultSolution():

    def __init__(self):
        self.drone = Drone(self.drone_instructions, self.process_frame, show_stream=True)

    def stop(self):
        self.drone.active = False
        self.drone.tello.end()

    def process_frame(self, frame):
        write_status_on_image(frame, get_full_drone_status(self.drone.tello))
        return frame

    def drone_instructions(self, tello):
        tello.takeoff()
        tello.move_up(150)
        tello.rotate_clockwise(360)
        tello.land()
    
    def main(self):
        self.drone.run_program()

    def stop(self):
        self.drone.active = False
        self.drone.tello.end()
