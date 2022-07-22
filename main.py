import sys
import signal
import argparse
from solutions.simple_use.use_with_lib import SimpleLibDemo
from solutions.simple_use.use_with_socket import SimpleSocketDemo
from solutions.simple_use.test_all_sensors import TestAllSensors
from solutions.following.follow_marker_coords import FollowMarkerWithCoords
from solutions.following.follow_marker_pixels import FollowMarkerWithPixels
from solutions.default import DefaultSolution
from solutions.calibration.calibrate_camera import CalibrateCamera


all_solutions = {
    "default": DefaultSolution,
    "simple_lib_demo": SimpleLibDemo,
    "simple_socket_demo": SimpleSocketDemo,
    "test_all_sensors_demo": TestAllSensors,
    "follow_marker_with_coords": FollowMarkerWithCoords,
    "follow_marker_with_pixels": FollowMarkerWithPixels,
    "calibrate_camera": CalibrateCamera,
}

sigint_counter = 0
solution = None


def signal_handler(sig, frame):
    global sigint_counter
    sigint_counter += 1
    if sigint_counter > 1 or not hasattr(solution, "stop"):
        print("Force interrupt execution")
        sys.exit(1)
    else:
        print("Carefully interrupt execution (Press Ctrl+C to force interrupt)")
        solution.stop()


def parse_args():
    parser = argparse.ArgumentParser(description="Drone solutions")
    parser.add_argument("-s", "--solution", choices=all_solutions.keys(), default="default", help="Solution for execution")
    args = parser.parse_args()
    return all_solutions[args.solution]


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    solution_class = parse_args()
    solution = solution_class()
    solution.main()
