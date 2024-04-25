from jsonrpcserver import method, Result, serve, Success, Error
from jsonrpcclient import request
import requests


SWARM = ["drone_1", "drone_2"]


@method(name="exec")
def exec(command: str) -> str:
    for node in SWARM:
        url = f"http://{node}:65000"
        req = request(url, "exec", command)
        response = requests.post(url, json=req)
        print(f"RESULT for '{url}' : {response}")
    return Success("OK")


if __name__ == "__main__":
    serve(port=65001)
