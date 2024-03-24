from enum import Enum, auto
import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyState:

    def __init__(self, msg: Joy) -> None:
        self.ButtonA = msg.buttons[0] == 1
        self.ButtonB = msg.buttons[1] == 1
        self.ButtonX = msg.buttons[2] == 1
        self.ButtonY = msg.buttons[3] == 1
        self.ButtonBack = msg.buttons[6] == 1
        self.ButtonStart = msg.buttons[7] == 1
        self.LStickH = msg.axes[0]
        self.LStickV = msg.axes[1]
        self.RStickH = msg.axes[3]
        self.RStickV = msg.axes[4]
        self.ButtonCrossL = msg.axes[6]  == -1
        self.ButtonCrossR = msg.axes[6]  == 1
        self.ButtonCrossD = msg.axes[7]  == -1
        self.ButtonCrossU = msg.axes[7]  == 1

    def convert_to_twist(self) -> Twist:
        msg = Twist()
        msg.linear.x = self.LStickV
        msg.linear.y = self.LStickH
        msg.linear.z = self.RStickV
        msg.angular.z = self.RStickH
        return msg


class TelloJoyNode(Node):

    def __init__(self):
        super().__init__("tello_joy_node")
        self.publisher = self.create_publisher(Bool, "/control", 10)
        self.publisher_twist = self.create_publisher(Twist, "cmd_vel", 10)
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.srv_action = self.create_client(TelloAction, "srv_tello_action")
        self.get_logger().info("Started tello_joy_node")

    def send_control_command(self, cmd: bool):
        msg = Bool()
        msg.data = cmd
        self.publisher.publish(msg)

    def send_action_command(self, cmd: str):
        request = TelloAction.Request()
        request.cmd = cmd
        self.srv_action.call_async(request)

    def joy_callback(self, msg):
        state = JoyState(msg)
        if state.ButtonStart:
            self.send_action_command("takeoff")
        if state.ButtonBack:
            self.send_action_command("land")
        if state.ButtonA:
            self.send_control_command(True)
        if state.ButtonB:
            self.send_control_command(False)
        self.publisher_twist.publish(state.convert_to_twist())
        # self.get_logger().info("Joy CB")
        # self.get_logger().info(str(msg))
        # self.get_logger().info(str(msg.axes))
        # self.get_logger().info(str(msg.buttons))


def main(args=None):
    rclpy.init(args=args)
    node = TelloJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
