from jsonrpcserver import method, Success, serve
from jsonrpcclient import request
import requests

SWARM = ["drone_1", "drone_2"]  # List of drones in the swarm

@method(name="exec")
def exec(command: str) -> str:
    """
    JSON-RPC method to execute a command on all drones in the swarm.
    """
    for node in SWARM:
        url = f"http://{node}:65000"
        req = request(url, "exec", command)
        response = requests.post(url, json=req)
        print(f"RESULT for '{url}' : {response}")

    # Return success response
    return Success("OK")

if __name__ == "__main__":
    serve(port=65001)
