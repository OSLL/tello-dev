from jsonrpcserver import method, Success, Error, serve

@method(name="exec")
def exec(command: str) -> str:
    """
    JSON-RPC method to execute a command.
    """
    status = "OK"  # Simulate command execution status

    # Print the command received
    print(command)

    # Return success or error response based on status
    return Success(command) if status == "OK" else Error(1, command)

if __name__ == "__main__":
    serve(port=65000)
