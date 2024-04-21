import json
import subprocess
import sys
from jsonschema import validate 
from jsonschema.validators import Draft202012Validator


class Networking:
    """
    Class for configuring network and routing based on a JSON configuration file.

    Attributes:
        PORTS (list): List of ports for routing.
    """

    PORTS = [8889, 8890, 11111]

    def __init__(self, config: dict):
        """
        Initializes the Networking class object.

        Args:
            config (dict): Network configuration in dictionary format.
        """
        self.__mark = 1
        for iface in config["ifaces"]:
            for port in Networking.PORTS:
                self.__routing(iface, port, config["ifaces"][iface]["ip"])
        self.__connection(config)

    def __connection(self, config: dict):
        """
        Establishes connection to Wi-Fi networks specified in the configuration.

        Args:
            config (dict): Network configuration in dictionary format.
        """
        for iface in config["ifaces"]:
            ssid = config['ifaces'][iface]['ssid']
            password = config['ifaces'][iface]['password']
            try:
                output = subprocess.check_output(
                    ['nmcli', 'device', 'wifi']).decode(encoding='utf-8').replace("\n", "")

                output = subprocess.check_output(
                    ['nmcli', 'device', 'wifi', 'connect', ssid, 'password',
                        password, 'ifname', iface]).decode(encoding='utf-8').replace("\n", "")
            except subprocess.CalledProcessError as e:
                print(f"Failed to connect to {ssid} on {iface}. Error: {e}")
            except Exception as e:
                print(
                    f"An error occurred while connecting to {ssid} on {iface}. Error: {e}")

    def __routing(self, iface: str, port: int, ip: str):
        """
        Configures routing for a specific network interface, port, and IP address.

        Args:
            iface (str): Network interface.
            port (int): Port for routing.
            ip (str): IP address for routing.
        """
        try:
            output = subprocess.check_output(
                ['iptables', '-t', 'nat', '-A', 'PREROUTING', '-i', iface, '-p',
                    'udp', '--dport', str(port), '-j', 'DNAT', '--to-destination', ip]
            ).decode(encoding='utf-8').replace("\n", "")

            output = subprocess.check_output(
                ['iptables', '-t', 'mangle', '-A', 'PREROUTING', '-s', ip, '-j', 'MARK',
                    '--set-mark', str(self.__mark)]).decode(encoding='utf-8').replace("\n", "")

            output = subprocess.check_output(
                ['ip', 'route', 'add', 'default', 'dev',
                    iface, 'table', str(self.__mark * 100)]).decode(encoding='utf-8').replace("\n", "")

            output = subprocess.check_output(
                ['ip', 'rule', 'add', 'fwmark', str(self.__mark),
                    'table', str(self.__mark * 100)]).decode(encoding='utf-8').replace("\n", "")

            self.__mark += 1
        except subprocess.CalledProcessError as e:
            print(f"Error occurred while setting up routing: {e}")
        except Exception as e:
            print(f"An error occurred while setting up routing: {e}")


def read_network_configuration(json_path):
    """
    Reads and validates the network configuration from a JSON file.

    Args:
        json_path (str): Path to the JSON file containing the network configuration.

    Returns:
        dict: Network configuration in dictionary format.
    """
    schema = {
        "type": "object",
        "properties": {
            "ifaces": {
                "type": "object",
                "patternProperties": {
                    "^[a-zA-Z0-9-]*$": {
                        "type": "object",
                        "properties": {
                            "ssid": {"type": "string"},
                            "password": {"type": "string"},
                            "ip": {"type": "string", "format": "ipv4"}
                        },
                        "required": ["ssid", "password", "ip"]
                    }
                }
            }
        },
        "required": ["ifaces"]
    }

    try:
        with open(json_path) as json_file:
            config = json.load(json_file)
            validate(instance=config, schema=schema,
                     format_checker=Draft202012Validator.FORMAT_CHECKER)
            return config
    except FileNotFoundError:
        print(f"File '{json_path}' not found.")
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from '{json_path}': {e}")
    except Exception as e:
        print(f"An error occurred while reading network configuration: {e}")
    return {}


def main(json_path):
    """
    Main function of the program.

    Args:
        json_path (str): Path to the JSON file containing the network configuration.
    """
    config = read_network_configuration(json_path)
    if config:
        networking = Networking(config)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <json_path>")
        sys.exit(1)
    main(sys.argv[1])
