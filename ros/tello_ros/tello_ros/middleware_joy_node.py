import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

class MiddlewareJoyNode(Node):

    QUEUE_SIZE = 10

    def __init__(self):
        super().__init__("middleware_joy_node")
        self.control = self.create_subscription(Bool, "/control", self.control_callback, self.QUEUE_SIZE)
        self.input_cmd_vel = self.create_subscription(Twist, "input_cmd_vel", self.input_cmd_vel_sub_callback, self.QUEUE_SIZE)
        self.output_cmd_vel = self.create_publisher(Twist, "output_cmd_vel", self.QUEUE_SIZE)
        self.output_srv_action = self.create_client(TelloAction, "output_tello_action")
        self.input_srv_action = self.create_service(TelloAction, 'input_tello_action', self.input_srv_callback)
        self.active_solution = False

    def input_cmd_vel_sub_callback(self, msg):
        if not self.active_solution:
            self.output_cmd_vel.publish(msg)

    def input_srv_callback(self, request, response):
        if not self.active_solution:
            self.output_srv_action.call_async(request)
        response.rc = 0  # TelloAction.OK
        return response

    def control_callback(self, msg):
        self.active_solution = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = MiddlewareJoyNode()
    rclpy.spin(node)
    # Unreachable if run with docker-compose
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
