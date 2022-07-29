import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction
from tello_ros.basic_tello_solution_node import BasicTelloSolutionNode

class TelloSimpleSolution(BasicTelloSolutionNode):

    def __init__(self):
        super().__init__("tello_publisher")
        self.timer = self.create_timer(1.5, self.timer_callback)

    def timer_callback(self):
        if not self.active:
            return
        self.send_twist_command(0.0, self.direction * 0.5 * self.skip_action, 0.0, 0.0)
        if self.skip_action != 0:
            self.direction *= -1
        self.skip_action = (self.skip_action + 1) % 2

    def run_solution(self):
        super().run_solution()
        self.direction = 1
        self.skip_action = 0


def main(args=None):
    rclpy.init(args=args)
    node = TelloSimpleSolution()
    rclpy.spin(node)
    # Unreachable if run with docker-compose
    node.stop_and_land_drone()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
