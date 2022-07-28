import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction


help_str = "Available commands:\n* [r]un -- run solution\n* [s]top -- stop solution"\
           "\n* takeoff -- take off the drone\n* land -- land the drone\n* shutdown -- shutdown manager node"


class ManagerNode(Node):

    def __init__(self):
        super().__init__("manager_node")
        self.publisher = self.create_publisher(Bool, "/control", 10)
        self.srv_action = self.create_client(TelloAction, "srv_tello_action")

    def send_control_command(self, cmd: bool):
        msg = Bool()
        msg.data = cmd
        self.publisher.publish(msg)

    def send_action_command(self, cmd: str):
        request = TelloAction.Request()
        request.cmd = cmd
        self.srv_action.call_async(request)

    def parse_and_send_command(self, string: str) -> bool:
        ret = True
        if string in ["r", "run"]:
            self.send_control_command(True)
        elif string in ["s", "stop"]:
            self.send_control_command(False)
        elif string == "takeoff":
            self.send_action_command("takeoff")
        elif string == "land":
            self.send_control_command(False)
            self.send_action_command("land")
        else:
            ret = False
        return ret


def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    print("ManagerNode\n")
    print(help_str)
    try:
        while True:
            s = input(">>> ")
            if s == "shutdown":
                print("Shutdown ManagerNode")
                break
            if not node.parse_and_send_command(s):
                print("Unknown command", s)
                print(help_str)
    except Exception as exception:
        print("Exception:", exception)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
