import cv2

FONT = cv2.FONT_HERSHEY_TRIPLEX
FONT_SIZE = 0.8
FONT_COLOR = (0, 255, 0)
FONT_STOKE = 2
LINE_STEP = 25

def write_status_on_image(image, status: dict, start_position=(0, LINE_STEP)):
    position = list(start_position)
    for key, val in status.items():
        cv2.putText(image, f"{key}: {val}", position, FONT, FONT_SIZE, FONT_COLOR, FONT_STOKE)
        position[1] += LINE_STEP
