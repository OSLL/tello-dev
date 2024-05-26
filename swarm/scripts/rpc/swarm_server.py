from jsonrpcserver import method, Success, serve
from jsonrpcclient import request
import requests

SWARM = ["drone_1", "drone_2"]  # List of drones in the swarm
all_drones_coord = []
move_marker_coord = []
@method(name="exec")
def exec(json: dict) -> str:
    """
    JSON-RPC method to execute a command on all drones in the swarm.
    """

    params = {
            "command": json["command"],
    }
    if json["command"] == "turn_starting_position":
        params["coordinates"] = json["coordinates"]
        all_drones_coord.append(json["coordinates"])
    if  json["command"] == "save_coord_move_marker":
        params["coordinates"] = json["coordinates"]
        params["coord_all_drones"]= all_drones_coord
        move_marker_coord.append(json["coordinates"])
    send_command_all_drones(params)

    if len(all_drones_coord) == len(SWARM):
        params = {
            "command": "set_swarm_is_coordinated"
        }
        send_command_all_drones(params)
        

    # Return success response
    return Success("OK")

def send_command_all_drones(params):
    for node in SWARM:
        url = f"http://{node}:65000"
        req = request("exec", params)
        response = requests.post(url, json=req)
        print(f"RESULT for '{url}' : {response}")

if __name__ == "__main__":
    serve(port=65001)
