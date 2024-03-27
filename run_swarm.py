import docker
from loguru import logger
from rtl_device.connect import readNetworkConfiguration


def start_containers(image_name, network_name, num_containers):
    client = docker.from_env()
    network = client.networks.get(network_name)

    for i in range(num_containers):
        container_name = f"{image_name}_{i+1}"
        container = client.containers.run(image=image_name, detach=True, name=container_name, network=network_name)
        logger.info(f"Container {container_name} started and attached to network {network_name}")

if __name__ == "__main__":
    config = readNetworkConfiguration("./rtl_device/networks.json")
    image_name = 'single_drone_swarm'
    network_name = list(config["docker_networks"].keys())[0]
    num_containers = 3

    start_containers(image_name, network_name, num_containers)
