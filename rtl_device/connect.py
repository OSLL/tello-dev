import json
import wifi
import subprocess
from loguru import logger
# Необходимо установить wireless-tools, ifupdown


def readNetworkConfiguration(json_path):
    with open(json_path) as json_file:
        return json.load(json_file)


def connect(iface, ssid, password) -> bool:
    logger.info(f"Connecting device \'{iface}\' to \'{ssid}\'")
    scanner = wifi.Cell.all(iface)
    available_networks = [cell.ssid for cell in scanner]
    if ssid not in available_networks:
        logger.error(f"SSID {ssid} unreacheble")
        return False
    try:
        output = subprocess.check_output(
            ['nmcli', 'device', 'wifi', 'connect', ssid, 'password', password, 'ifname', iface]).decode(encoding='utf-8').replace("\n", "")

        if "successfully" in str(output):
            logger.success(f'Device \'{iface}\' successfully connected to \'{ssid}\'')
            return True
        else:
            logger.error(f"Error while connecting to {ssid}")
            logger.error(output)
            return False
    except Exception as e:
        logger.critical(e)
        logger.debug(output)

    return False


if __name__ == "__main__":
    ifaces = readNetworkConfiguration("networks.json")["ifaces"]
    for iface in ifaces:
        connect(iface, ifaces[iface]['ssid'], ifaces[iface]['password'])
