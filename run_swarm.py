import docker
from loguru import logger
from rtl_device.connect import readNetworkConfiguration

def createDockerNetwork(name: str) -> None:
    client = docker.from_env()
    client.networks.create(name)

def start_containers(image_name, network_name, num_containers):
    client = docker.from_env()
    try:
        network = client.networks.get(network_name)
    except docker.errors.NotFound:
        logger.warning(f"Network '{network_name}' not found, creating it...")
        createDockerNetwork(network_name)  # Создаем сеть
        network = client.networks.get(network_name)

    for i in range(num_containers):
        container_name = f"{image_name}_{i+1}"

        try:
            old_container = client.containers.get(container_name)
            logger.info(f"Removing old container: {container_name}")
            old_container.remove(force=True)
        except docker.errors.NotFound:
            pass

        container = client.containers.run(image=image_name, detach=True, name=container_name, network=network_name, stdout=True)
        logger.info(f"Container {container_name} started and attached to network {network_name}")

        container_logs = container.logs().decode("utf-8")
        logger.info(f"Container logs for {container_name}:")
        logger.info(container_logs)

if __name__ == "__main__":
    config = readNetworkConfiguration("./rtl_device/networks.json")
    image_name = 'single_drone_swarm'
    network_name = list(config["docker_networks"].keys())[0]
    num_containers = 3

    start_containers(image_name, network_name, num_containers)

