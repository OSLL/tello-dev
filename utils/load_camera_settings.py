import os
import numpy as np


def load_camera_settings(filename="camera_settings.npy"):
    if not os.path.exists(filename):
        print(f"File {filename} with camera settings not exists!")
        mtx = np.zeros((3, 3))
        dist = np.zeros(5)
    else:
        with open(filename, "rb") as f:
            mtx = np.load(f)
            dist = np.load(f)
    return mtx, dist
