import cv2
import time
import math
import numpy as np
from solutions.following.state import State
from solutions.following.follow_marker_coords import FollowMarkerWithCoords
from solutions.following.config import X_RANGE, Y_RANGE, STEP, Z_STEP, MARKER_SIZE,\
    MARKER_SIZE_RANGE


class FollowMarkerWithPixels(FollowMarkerWithCoords):

    def __init__(self):
        super().__init__()
        self.center = (0, 0)

    def get_state(self, center, marker_size, m_point):
        x_diff = m_point[0] - center[0] # diff on frame
        y_diff = m_point[1] - center[1] # diff on frame
        x = y = z = 0
        if x_diff > X_RANGE:
            y = -STEP
        elif x_diff < -X_RANGE:
            y = STEP
        if y_diff > Y_RANGE:
            z = -Z_STEP
        elif y_diff < -Y_RANGE:
            z = Z_STEP
        if marker_size < MARKER_SIZE - MARKER_SIZE_RANGE:
            x = STEP
        elif marker_size > MARKER_SIZE + MARKER_SIZE_RANGE:
            x = -STEP
        if x == 0 and y == 0 and z == 0:
            return None
        return (State.MOVE, np.array([x, y, z], dtype=np.int32))

    def move_to_marker(self, m_corner):
        m_corner = m_corner[0]
        marker_size = max(np.sqrt(np.sum((m_corner - np.roll(m_corner, (1, 1))) ** 2, 1)))
        marker_center = (
            (max(m_corner[:, 0]) + min(m_corner[:, 0])) // 2,
            (max(m_corner[:, 1]) + min(m_corner[:, 1])) // 2
        )
        state = self.get_state(self.center, marker_size, marker_center)
        if state is not None:
            self.actions.put(state)
            self.last_state = state

    def process_frame(self, frame):
        frame = super().process_frame(frame)
        self.center = (frame.shape[1] // 2, frame.shape[0] // 2)
        center = self.center
        frame = cv2.rectangle(frame, (center[0] - X_RANGE, center[1] - Y_RANGE), (center[0] + X_RANGE, center[1] + Y_RANGE), (0, 255, 0), 2)
        return frame
