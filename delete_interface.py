import subprocess

def delete_docker_interface(interface_name):
    subprocess.run(["ip", "link", "delete", interface_name])
    print(f"Interface {interface_name} deleted.")

def input_interface():
    interface_name = input("Enter the interface name to delete: ")
    delete_docker_interface(interface_name)

if __name__ == '__main__':
    input_interface()