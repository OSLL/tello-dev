import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tello_msgs.msg import FlightData
from tello_ros.config import mtx, dist, MARKER_LEN, MARKER_NUMBER
from tello_ros.basic_tello_control_node import BasicTelloControlNode

class ArUcoMarkerFollower(BasicTelloControlNode):

    FONT = cv2.FONT_HERSHEY_TRIPLEX
    FONT_SIZE = 1
    FONT_COLOR = (0, 255, 0)
    FONT_STOKE = 2
    LINE_STEP = 30

    def __init__(self):
        super().__init__("aruco_marker_follower")
        self.cv_bridge = CvBridge()
        self.cur_status = [
            "battery: u%",
            "height: 0cm",
            "flight time: 0s",
            "pitch: 0",
            "roll: 0",
            "yaw: 0",
        ]
        self.input_img_sub = self.create_subscription(Image, "input_images", self.process_image, 10)
        self.status_sub = self.create_subscription(FlightData, "flight_status", self.update_status, 10)
        self.out_img_pub = self.create_publisher(Image, "output_images", 10)
        self.timer = self.create_timer(1.5, self.timer_callback)  # For testing, delete this
        self.direction = 1
        self.skip_action = 1

    # For testing, delete this
    def timer_callback(self):
        if not self.active:
            return
        self.send_twist_command(0.0, self.direction * 0.5 * self.skip_action, 0.0, 0.0)
        if self.skip_action != 0:
            self.direction *= -1
        self.skip_action = (self.skip_action + 1) % 2

    def process_image(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.write_status_on_image(img)
        new_img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.out_img_pub.publish(new_img_msg)

    def write_status_on_image(self, img):
        pos = [0, self.LINE_STEP]
        for val in self.cur_status:
            cv2.putText(img, val, pos, self.FONT, self.FONT_SIZE, self.FONT_COLOR, self.FONT_STOKE)
            pos[1] += self.LINE_STEP

    def update_status(self, msg):
        new_status = [
            f"battery: {msg.bat}%",
            f"height: {msg.h}cm",
            f"flight time: {msg.time}s",
            f"pitch: {msg.pitch}",
            f"roll: {msg.roll}",
            f"yaw: {msg.yaw}",
        ]
        self.cur_status = new_status


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
