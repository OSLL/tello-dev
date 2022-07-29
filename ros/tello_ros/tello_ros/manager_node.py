from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tello_msgs.srv import TelloAction


help_str = "Available commands:\n"\
           "* [r]un -- run solution\n"\
           "* [s]top -- stop solution\n"\
           "* takeoff -- take off the drone\n"\
           "* land -- land the drone\n"\
           "* call <cmd> -- send <cmd> to the drone (rc command is not allowed)\n"\
           "* shutdown -- shutdown manager node"


class Result(Enum):
    OK = auto()
    UNKNOWN = auto()
    PROHIBITION = auto()


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
        cmd = string.strip()
        ret = Result.OK
        if cmd in ["r", "run"]:
            self.send_control_command(True)
        elif cmd in ["s", "stop"]:
            self.send_control_command(False)
        elif cmd in ["takeoff", "land"]:
            self.send_control_command(False)
            self.send_action_command(cmd)
        elif cmd.startswith("call "):
            cmd = cmd[5:]
            if cmd.find("rc") != -1:
                ret = Result.PROHIBITION
            else:
                self.send_action_command(cmd)
        else:
            ret = Result.UNKNOWN
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
            res = node.parse_and_send_command(s)
            if res == Result.UNKNOWN:
                print("Unknown command", s)
                print(help_str)
            elif res == Result.PROHIBITION:
                print("Command", s, "is not allowed")
    except Exception as exception:
        print("Exception:", exception)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
