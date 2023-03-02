import time
import math
import rclpy
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tello_msgs.msg import FlightData
from tello_ros.config import mtx, dist, MARKER_LEN, MARKER_NUMBER, ARUCO_DICT, ARUCO_PARAMS
from tello_ros.basic_tello_solution_node import BasicTelloSolutionNode
from enum import Enum, auto


class Action(Enum):
    STOP = auto()
    MOVE = auto()
    FLIP = auto()
    BACK = auto()
    NONE = auto()


class ArUcoMarkerFollower(BasicTelloSolutionNode):

    FONT = cv2.FONT_HERSHEY_TRIPLEX
    FONT_SIZE = 1
    FONT_COLOR = (0, 255, 0)
    FONT_STOKE = 2
    LINE_STEP = 30
    TIMEOUT = 0.05
    EPS = 0.02
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
                static_image_mode=True,
                max_num_hands=1,
                min_detection_confidence=0.5)

    def __init__(self):
        super().__init__("aruco_marker_follower")
        self.cv_bridge = CvBridge()
        self.cur_status = {
            "battery(%)": "u",
            "height(cm)": 0,
            "flight time(s)": 0,
            "pitch": 0,
            "roll": 0,
            "yaw": 0,
        }
        self.input_img_sub = self.create_subscription(Image, "input_images", self.process_image, 10)
        self.status_sub = self.create_subscription(FlightData, "flight_status", self.update_status, 10)
        self.out_img_pub = self.create_publisher(Image, "output_images", 10)
        self.last_marker_time = time.process_time()
        self.draw_marker = True
        self.speed_coeff = 0.3

        self.height = 20
        self.distance = 90
        self.decrease_rotation_coeff = 2.0
        self.rotation_speed = 90 / self.decrease_rotation_coeff
        self.flip_timeout = 0.5

    def process_image(self, msg):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            # res = self.find_marker(img)
            res = self.find_hand(img)
            if res is not None and self.draw_marker:
                # img = cv2.aruco.drawDetectedMarkers(img, np.array([res[2]]), np.array([res[3]]))
                img = self.draw_hand(img, res)
            if self.active:
                if res is not None:
                    # self.fly_to_marker(res[0], res[1])
                    self.action_from_hand(img, res)
                    self.last_marker_time = time.process_time()
                elif time.process_time() - self.last_marker_time > self.TIMEOUT:
                    self.send_twist_command(0.0, 0.0, 0.0, 0.0)
                    self.last_marker_time = time.process_time()
            self.write_status_on_image(img)
            new_img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
            self.out_img_pub.publish(new_img_msg)
        except Exception as e:
            self.get_logger().info(f"Ошибка {e}")

    def write_status_on_image(self, img):
        pos = [0, self.LINE_STEP]
        for key, val in self.cur_status.items():
            cv2.putText(img, f"{key}: {val}", pos, self.FONT, self.FONT_SIZE, self.FONT_COLOR, self.FONT_STOKE)
            pos[1] += self.LINE_STEP
    def update_status(self, msg):
        new_status = {
            "battery(%)": msg.bat,
            "height(cm)": msg.h,
            "flight time(s)": msg.time,
            "pitch": msg.pitch,
            "roll": msg.roll,
            "yaw": msg.yaw,
        }
        self.cur_status = new_status

    def detect_marker_on_image(self, image):
        corners, ids, rejected = cv2.aruco.detectMarkers(image, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None:
            for i in range(len(ids)):
                if ids[i] == MARKER_NUMBER:
                    return (ids[i], corners[i])
        return None

    def find_hand(self, image):
        image.flags.writeable = False
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return results.multi_hand_landmarks

    def check_ok(self, results):
        for hand_landmarks in results:
            landmark = hand_landmarks.landmark
            v = np.array([landmark[4].x - landmark[8].x, landmark[4].y - landmark[8].y])
            d = np.linalg.norm(v)
            if d < self.EPS / 2:
                self.send_tello_action_async("land")
                return True
            return False
        return False


    def action_from_hand(self, img, results):
        if results:
            for hand_landmarks in results:
                # if self.check_ok(results):
                #     cv2.putText(img, f"Landing", (0, self.LINE_STEP * (len(self.cur_status) + 1)), self.FONT, self.FONT_SIZE, self.FONT_COLOR, self.FONT_STOKE)
                #     return
                vector = self.get_fingers_vectors(hand_landmarks.landmark)
                count = 0
                for y in vector:
                    count += 1 if y < -self.EPS else 0
                # if count >= 4:
                #     action = Action.STOP
                #     self.get_logger().info(f"Flying with speeds (0.0, 0.0, 0.0, 0.0)")
                #     self.send_twist_command(0.0, 0.0, 0.0, 0.0)
                if count == 1:
                    # action = Action.BACK
                    # self.get_logger().info(f"Flying with speeds ({-self.speed_coeff}, 0.0, 0.0, 0.0)")
                    # self.send_twist_command(-self.speed_coeff, 0.0, 0.0, 0.0)
                    action = Action.BACK
                    self.get_logger().info(f"Flying with speeds ({-self.speed_coeff}, 0.0, 0.0, 0.0)")
                    self.send_twist_command(-self.speed_coeff, 0.0, 0.0, 0.0)
                elif count == 2:
                    # action = Action.FLIP
                    # if time.process_time() - self.last_marker_time > self.flip_timeout:
                    #     self.send_tello_action_async("flip b")
                    action = Action.MOVE
                    self.get_logger().info(f"Flying with speeds ({self.speed_coeff}, 0.0, 0.0, 0.0)")
                    self.send_twist_command(self.speed_coeff, 0.0, 0.0, 0.0)
                elif count == 0:
                    action = Action.STOP
                    self.get_logger().info(f"Flying with speeds (0.0, 0.0, 0.0, 0.0)")
                    self.send_twist_command(0.0, 0.0, 0.0, 0.0)
                else:
                    action = Action.NONE
                cv2.putText(img, f"Action is {action}(fingers {count})", (0, self.LINE_STEP * (len(self.cur_status) + 1)), self.FONT, self.FONT_SIZE, self.FONT_COLOR, self.FONT_STOKE)


    def draw_hand(self, image, results):
        for hand_landmarks in results:
            self.mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style())
        return image

    def get_fingers_vectors(self, landmarks):
        # h, w = img_shape
        fingers_indecies = [
            (5, 8),
            (9, 12),
            (13, 16),
            (17, 20)
        ]
        return [
            landmarks[j].y - landmarks[i].y
            for i, j in fingers_indecies
        ]


    def hand_process_image(image: "np.ndarray"):
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
            # for land in hand_landmarks.landmark:
            #     print(land.x, land.y)
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
                vector = get_fingers_vectors(hand_landmarks.landmark)
                count = 0
                # print(vector)
                for y in vector:
                    count += 1 if y < -EPS else 0
                cv2.putText(image, f"fingers up {count}", (30, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2)

    def find_marker(self, image):
        res = self.detect_marker_on_image(image)
        if res is None:
            return None
        marker_id, marker_corner = res
        rvecs , tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([marker_corner]), MARKER_LEN, mtx, dist)
        return (tvecs[0][0], rvecs[0][0], marker_corner, marker_id)

    def fly_to_marker(self, tvec, rvec):
        # marker_[x,y,z] in drone axis
        marker_x, marker_y, marker_z = tvec[2], tvec[0], tvec[1]
        r_x = math.degrees(rvec[1])
        r_x_sign = r_x / abs(r_x)
        z = self.height - self.cur_status.get("height(cm)", self.height)
        y = -marker_y
        x = marker_x - self.distance
        if abs(r_x) > 10:
            r = self.rotation_speed * r_x_sign * 1.2
            y = math.pi * marker_x / 4 / self.decrease_rotation_coeff * 1.25
            y *= r_x_sign
        else:
            r = 0.0
            y = -marker_y
        x = max(-1.0, min(1.0, x / 100 * self.speed_coeff))
        y = max(-1.0, min(1.0, y / 100 * self.speed_coeff))
        z = max(-1.0, min(1.0, z * 2 / 100 * self.speed_coeff))
        r = max(-1.0, min(1.0, r / 100 * self.speed_coeff))
        self.get_logger().info(f"Flying with speeds ({x}, {y}, {z}, {r})")
        self.send_twist_command(x, y, z, -r)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoMarkerFollower()
    rclpy.spin(node)
    # Unreachable if run with docker-compose
    node.stop_and_land_drone()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
