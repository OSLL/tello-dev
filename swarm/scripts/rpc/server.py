from jsonrpcserver import method, Result, serve, Success, Error


@method(name="exec")
def exec(command: str) -> str:
    status = "OK"
    print(command)
    return Success(command) if status == "OK" else Error(1, command)


if __name__ == "__main__":
    serve(port=65000)
