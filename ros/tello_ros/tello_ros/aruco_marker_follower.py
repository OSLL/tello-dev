import time
import math
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tello_msgs.msg import FlightData
from tello_ros.config import mtx, dist, MARKER_LEN, MARKER_NUMBER, ARUCO_DICT, ARUCO_PARAMS
from tello_ros.basic_tello_solution_node import BasicTelloSolutionNode

class ArUcoMarkerFollower(BasicTelloSolutionNode):

    FONT = cv2.FONT_HERSHEY_TRIPLEX
    FONT_SIZE = 1
    FONT_COLOR = (0, 255, 0)
    FONT_STOKE = 2
    LINE_STEP = 30
    TIMEOUT = 0.8

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
        self.speed_coeff = 1.0
        self.height = 40
        self.distance = 100
        self.decrease_rotation_coeff = 2.0
        self.rotation_speed = 90 / self.decrease_rotation_coeff

    def process_image(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        res = self.find_marker(img)
        if res is not None and self.draw_marker:
            img = cv2.aruco.drawDetectedMarkers(img, np.array([res[2]]), np.array([res[3]]))
        if self.active:
            if res is not None:
                self.fly_to_marker(res[0], res[1])
                self.last_marker_time = time.process_time()
            elif time.process_time() - self.last_marker_time > self.TIMEOUT:
                self.send_twist_command(0.0, 0.0, 0.0, 0.0)
                self.last_marker_time = time.process_time()
        self.write_status_on_image(img)
        new_img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.out_img_pub.publish(new_img_msg)

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
