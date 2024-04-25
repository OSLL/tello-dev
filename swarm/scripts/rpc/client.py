import argparse
import requests
from jsonrpcclient import request


def main():
    """
    CLI application to read a command from the user.
    """
    parser = argparse.ArgumentParser(
        description='CLI application to read a command')
    parser.add_argument('-c', '--command', type=str, help='Command to execute')
    parser.add_argument('--ip', type=str, help='IP address')
    args = parser.parse_args()

    if args.command:
        if args.ip:
            url = f"http://{args.ip}:65001"
            req = request("exec", params={ "command": args.command} )
            response = requests.post(url, json=req)
            print(f"RESULT for '{url}' :\n\t {response.json()}")
        else:
            print('No IP address provided.')
    else:
        print('You need to pass the --command argument with a command to execute.')


if __name__ == "__main__":
    main()
