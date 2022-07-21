import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction

class TelloPublisher(Node):

    def __init__(self):
        super().__init__("tello_publisher")
        self.publisher = self.create_publisher(Twist, "/tello/cmd_vel", 10)
        self.control_subscriber = self.create_subscription(Bool, "/control", self.control_sub_callback ,10)
        self.cli = self.create_client(TelloAction, "/tello/tello_action")
        self.start_node()
        self.timer = self.create_timer(1.5, self.timer_callback)

    def timer_callback(self):
        if not self.active:
            return
        self.send_twist_command(0.0, self.direction * 0.5 * self.skip_action, 0.0, 0.0)
        if self.skip_action != 0:
            self.direction *= -1
        self.skip_action = (self.skip_action + 1) % 2

    def control_sub_callback(self, msg):
        self.get_logger().info(f"Received from /control {msg.data}")
        if msg.data and not self.active:
            self.start_node()
            self.get_logger().info(f"Continue work")
        elif not msg.data and self.active:
            self.stop_and_land_drone()
            self.get_logger().info(f"Work is stopped")

    def send_twist_command(self, x, y, z, rotation):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = rotation
        self.publisher.publish(msg)

    def send_request(self, cmd, wait=True):
        self.req.cmd = cmd
        future = self.cli.call_async(self.req)
        if wait:
            rclpy.spin_until_future_complete(self, future)
            return future.result().rc
        else:
            return 1

    def stop_and_land_drone(self):
        self.active = False
        self.send_twist_command(0.0, 0.0, 0.0, 0.0)
        while self.send_request("land") != 1:
            self.get_logger().info("Error on sending land")

    def start_node(self):
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Wait for service")
        self.req = TelloAction.Request()
        while self.send_request("takeoff") != 1:
            self.get_logger().info("Error on sending takeoff")
            time.sleep(2)
        self.direction = 1
        self.skip_action = 0
        self.active = True


def main(args=None):
    rclpy.init(args=args)
    p = TelloPublisher()
    rclpy.spin(p)
    p.stop_and_land_drone()
    p.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
