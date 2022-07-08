import sys
import signal
import following.following as flw
import following.distance as dst


sigint_counter = 0
main = None
stop = None


def signal_handler(sig, frame):
    global sigint_counter
    print("Interrupt execution")
    sigint_counter += 1
    if sigint_counter > 1 or stop is None:
        sys.exit(1)
    else:
        stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    if len(sys.argv) < 2:
        print("No args, terminating")
        sys.exit(2)
    cmd = sys.argv[1]
    if cmd == "following":
        stop = flw.stop
        main = flw.main
    elif cmd == "distance":
        main = dst.main()
    else:
        print("Incorrect command, terminating")
        sys.exit(2)
    main()
