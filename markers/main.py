import signal
import following.following as flw


def signal_handler(sig, frame):
    print("Interrupt execution")
    flw.stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    stop = flw.stop
    main = flw.main
    main()
