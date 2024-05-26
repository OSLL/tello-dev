from jsonrpcserver import method, Success, Error, serve
from swarm.scripts.solutions.following.swarm_controlls import FollowMarkerWithCoords

# Инициализация объекта дрона
drone_controller = FollowMarkerWithCoords()
drone_controller.main()

@method(name="exec")
def exec(json: dict) -> str:
    """
    JSON-RPC method to execute a command.
    """
    status = "OK"  # Simulate command execution status

    # Print the command received
    print(json["command"])

    if json["command"] == "turn_starting_position":
        drone_controller.turn_starting_position(json["coordinates"])
    if json["command"] == "land":
        drone_controller.land()
    if json["command"] == "set_swarm_is_coordinated":
        drone_controller.set_swarm_is_coordinated()
    if json["command"] == "save_coord_move_marker":
        drone_controller.save_move_marker(json["coordinates"])
        drone_controller.save_coord_all_drones(json["coord_all_drones"])
    if json["command"] == "set_rotate":
        drone_controller.set_rotate_flag()
    if json["command"] == "reset_rotate":
        drone_controller.reset_rotate_flag()
    # Return success or error response based on status
    return Success(json["command"]) if status == "OK" else Error(1, json["command"])

if __name__ == "__main__":
    serve(port=65000)
