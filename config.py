from utils.load_camera_settings import load_camera_settings

camera_matrix, camera_distortion = load_camera_settings()

print(camera_matrix)
print(camera_distortion)
MARKER_LEN = 6.5  # in cm
