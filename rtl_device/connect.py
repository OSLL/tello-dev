import json
import wifi
import subprocess
import netifaces
import docker
import sys
from loguru import logger
from typing import NoReturn


def readNetworkConfiguration(json_path):
    with open(json_path) as json_file:
        return json.load(json_file)


def connect(iface: str, ssid: str, password: str) -> bool:
    logger.info(f"Connecting device \'{iface}\' to \'{ssid}\'")
    scanner = wifi.Cell.all(iface)
    available_networks = [cell.ssid for cell in scanner]

    output = ""
    if ssid not in available_networks:
        logger.error(f"SSID {ssid} unreacheble")
        return False
    try:
        output = subprocess.check_output(
            ['nmcli', 'device', 'wifi', 'connect', ssid, 'password', password, 'ifname', iface]).decode(encoding='utf-8').replace("\n", "")
        if "successfully" in str(output):
            logger.info(netifaces.ifaddresses(iface))
            logger.success(
                f'Device \'{iface}\' successfully connected to \'{ssid}\'')
            return True
        else:
            logger.error(f"Error while connecting to {ssid}")
            logger.error(output)
            return False
    except Exception as e:
        logger.critical(e)
        logger.debug(output)
        sys.exit(-1)

    return False


def createDockerNetwork(name: str, iface: str, gateway: str, subnet: str) -> NoReturn:
    options = dict({("parent", iface)})
    srv_pool = docker.types.IPAMPool(subnet=subnet, gateway=gateway)
    ipam = docker.types.IPAMConfig(pool_configs=[srv_pool])
    client = docker.from_env()
    docker_networks = client.networks.list([name])
    if docker_networks != []:
        logger.warning(
            f"Network with name {name} already exists, removing it...")
        network = client.networks.get(docker_networks[0].id)
        network.remove()
        logger.info(f'Removed network with id {docker_networks[0].id}')
    logger.info(f'Creating network with name {name}')
    client.networks.create(name, "macvlan", options, ipam)


if __name__ == "__main__":
    config = readNetworkConfiguration("networks.json")
    ifaces = config["ifaces"]
    docker_networks = config["docker_networks"]

    for iface in ifaces:
        connect(iface, ifaces[iface]['ssid'], ifaces[iface]['password'])
    for network in docker_networks:
        createDockerNetwork(network, docker_networks[network]["iface"],
                            docker_networks[network]["gateway"],
                            docker_networks[network]["subnet"])
