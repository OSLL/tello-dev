import sys
import signal
import following.following as flw


sigint_counter = 0


def signal_handler(sig, frame):
    global sigint_counter
    print("Interrupt execution")
    sigint_counter += 1
    if sigint_counter > 1:
        sys.exit(1)
    else:
        flw.stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    stop = flw.stop
    main = flw.main
    main()
