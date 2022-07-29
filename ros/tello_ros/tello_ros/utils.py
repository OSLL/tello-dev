import yaml
import numpy as np


def read_camera_settings(filename):
    with open(filename, "r") as f:
        data = yaml.safe_load(f)
    mtx = np.array(data["camera_matrix"]["data"]).reshape((3, 3))
    dst = np.array(data["distortion_coefficients"]["data"])
    return mtx, dst
