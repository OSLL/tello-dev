# Swarm

This project consists of three Python scripts: client.py, server.py, and swarm_server.py. 

## client.py

This script is a CLI application that sends commands to a JSON-RPC server. It accepts two command-line arguments:
- -c or --command: The command to execute.
- --ip: The IP address of the JSON-RPC server.

> Example usage:
python client.py -c "ls -l" --ip "127.0.0.1"

## server.py

This script implements a JSON-RPC server with a single method exec, which executes the provided command.

**Method exec:**
- Description: 
    - Executes the provided command.
- Parameters:
    - command (str): The command to execute.
- Returns:
    - str: The executed command.

**Request:**
```json
{
    "jsonrpc": "2.0",
    "method": "exec",
    "params": {
        "command": "takeoff"
    },
    "id": 1
}
```
**Response:**
```json
  {
      "jsonrpc": "2.0",
      "result": "OK",
      "id": 1
  }
```
# swarm_server.py

This script implements a JSON-RPC server similar to server.py, but it is designed to work in a swarm environment. It executes the provided command on all drones in the swarm.

**Method exec:**
- Description: Executes the provided command on all drones in the swarm.
- Parameters:
  - command (str): The command to execute.
- Returns:
  - str: The status of the execution.

**Request:**
```json
  {
      "jsonrpc": "2.0",
      "method": "exec",
      "params": {
          "command": "ls -l"
      },
      "id": 1
  }
```
**Response:**
```json
  {
      "jsonrpc": "2.0",
      "result": "OK",
      "id": 1
  }
```

# networking.py - Networking Class

Class for configuring network and routing based on a JSON configuration file.

## Attributes
- `PORTS` (list): List of ports for routing.

## Methods

### \_\_init\_\_(config: dict)
Initializes the Networking class object.

**Parameters:**
- `config` (dict): Network configuration in dictionary format.

### \_\_connection(config: dict)
Establishes connection to Wi-Fi networks specified in the configuration.

**Parameters:**
- `config` (dict): Network configuration in dictionary format.

### \_\_routing(iface: str, port: int, ip: str)
Configures routing for a specific network interface, port, and IP address.

**Parameters:**
- `iface` (str): Network interface.
- `port` (int): Port for routing.
- `ip` (str): IP address for routing.

## read_network_configuration(json_path: str) -> dict
Reads and validates the network configuration from a JSON file.

**Parameters:**
- `json_path` (str): Path to the JSON file containing the network configuration.

**Returns:**
- dict: Network configuration in dictionary format.

## main(json_path: str)
Main function of the program.

**Parameters:**
- `json_path` (str): Path to the JSON file containing the network configuration.


# Docker Files and Docker Compose

## Dockerfile.networking

This Dockerfile is used to build the networking container. It installs necessary networking tools and sets up the environment for networking.py.

## Dockerfile.swarm_node

This Dockerfile is used to build a swarm node container. It installs Python and the required dependencies for JSON-RPC server.

## Dockerfile.swarm_server

This Dockerfile is used to build the swarm server container. It installs Python and the required dependencies for JSON-RPC server, JSON-RPC client, and requests library.

## docker-compose.yml

This Docker Compose file defines the services required for the JSON-RPC application. It includes the networking service, server service, and two drone services. The networking service is configured to run with host network mode and provide necessary network functionalities for the application. The server service is configured to depend on the networking service and two drone services. It exposes port 65001 for external access. The drone services are configured to depend on the networking service and provide IP addresses for the swarm network.
