import docker

def run_container(image_name, container_name, requirements_file):
    client = docker.from_env()
    try:
        # Считываем содержимое файла requirements.txt
        with open(requirements_file, 'r') as file:
            requirements = file.read().splitlines()

        # Строим контейнер
        container = client.containers.run(
            image=image_name,
            name=container_name,
            detach=True,
            volumes={requirements_file: {'bind': '/requirements.txt', 'mode': 'ro'}},
            command=f"sh -c 'pip install -r /requirements.txt && tail -f /dev/null'"
        )
        print(f"Container {container_name} started successfully.")
    except docker.errors.APIError as e:
        print(f"Error starting container: {e}")

if __name__ == "__main__":
    image_name = "python"
    container_name = "droner"
    requirements_file = "requirements.txt"

    run_container(image_name, container_name, requirements_file)
