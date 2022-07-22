import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse


class BasicTelloControlNode(Node):

    QUEUE_SIZE = 10

    def __init__(self, node_name):
        super().__init__(node_name)
        self.cmd_vel_pub = self.create_publisher(Twist, "out_cmd_vel", self.QUEUE_SIZE)
        self.control_sub= self.create_subscription(Bool, "/control", self.control_sub_callback, self.QUEUE_SIZE)
        self.srv_action = self.create_client(TelloAction, "srv_tello_action")
        self.srv_sub = self.create_subscription(TelloResponse, "action_response", self.response_for_action_callback, self.QUEUE_SIZE)
        self.active = False
        self.received_response = None

    def control_sub_callback(self, msg):
        if self.active is None:
            self.get_logger().info(f"Received {msg.data} from /control, during node starting. Skipping message")
            return
        self.get_logger().info(f"From /control received {msg.data}")
        if msg.data and not self.active:
            self.start_node()
            self.get_logger().info(f"Continue work")
        elif not msg.data and self.active:
            self.stop_and_land_drone()
            self.get_logger().info(f"Work is stopped")

    def send_twist_command(self, x, y, z, rotation):
        if not isinstance(x, float):
            self.get_logger().info("Variable x is not float. Do not send command")
            return
        if not isinstance(y, float):
            self.get_logger().info("Variable y is not float. Do not send command")
            return
        if not isinstance(z, float):
            self.get_logger().info("Variable z is not float. Do not send command")
            return
        if not isinstance(rotation, float):
            self.get_logger().info("Variable rotation is not float. Do not send command")
            return
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = rotation
        self.cmd_vel_pub.publish(msg)

    def send_tello_action(self, cmd):
        if self.received_response is not None:
            return False
        while not self.srv_action.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Wait for service")
        while True:
            ret = self.__send_srv_action(cmd)
            if ret == 1:
                break
            elif ret == 2:
                self.get_logger().info(f"Drone is unavailable")
                time.sleep(2)
            else:
                self.get_logger().info(f"Error on sending action '{cmd}'")
        # Code is not working
        #
        # Replace `while self.received_response is None` with something more cool
        # while self.received_response is None:
        #     pass
        #
        # if self.received_response.lower().find("ok") != -1:
        #     ret = True
        # else:
        #     self.get_logger().info(f"Error on sending action '{cmd}'. Response: {self.received_response}")
        #     ret = False
        # self.get_logger().info(f"response is {self.received_response}")
        # self.received_response = None
        # return ret
        return True

    def __send_srv_action(self, cmd):
        request = TelloAction.Request()
        request.cmd = cmd
        future = self.srv_action.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().rc

    def response_for_action_callback(self, response):
        self.get_logger().info(f"Received response for tello action")
        # It is for not working code
        #
        # self.received_response = response.str

    def stop_and_land_drone(self):
        self.active = False
        self.send_twist_command(0.0, 0.0, 0.0, 0.0)
        while not self.send_tello_action("land"):
            self.get_logger().info("Error on sending land")

    def start_node(self):
        while not self.send_tello_action("takeoff"):
            self.get_logger().info("Error on sending takeoff")
        self.active = True
