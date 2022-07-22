import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from tello_msgs.msg import FlightData


class ImageProcessing(Node):

    FONT = cv2.FONT_HERSHEY_TRIPLEX
    FONT_SIZE = 1
    FONT_COLOR = (0, 255, 0)
    FONT_STOKE = 2
    LINE_STEP = 30

    def __init__(self):
        super().__init__("image_subscriber")
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
    node = ImageProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
