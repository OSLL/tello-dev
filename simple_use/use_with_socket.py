import cv2
import time
import threading
import socket


ip = "192.168.10.1"
control_port = 8889
status_port = 8890

active = True


def drone_video(backreader):
    try:
        while active:
            # Read frame
            ret, cur_frame = backreader.read()
            if ret is None:
                continue
            cv2.imshow("Drone stream", cur_frame)
            cv2.waitKey(1)
    except Exception as exception:
        print("Exception in drone_video:", exception)


def send_control_command(udp_socket, command: str):
    ok = False
    while not ok:
        udp_socket.sendto(bytes(command, "utf-8"), (ip, control_port))
        data, addr = udp_socket.recvfrom(1024)
        if data.decode("utf-8") == "ok":
            ok = True


def send_status_command(udp_socket, command: str):
    udp_socket.sendto(bytes(command, "utf-8"), (ip, control_port))
    data, addr = udp_socket.recvfrom(1024)
    return data.decode("utf-8").replace("\r\n", "")


def drone_prog(udp_socket):
    send_control_command(udp_socket, "takeoff")
    send_control_command(udp_socket, "forward 100")
    time.sleep(1)
    send_control_command(udp_socket, "up 100")
    time.sleep(1)
    time.sleep(1)
    send_control_command(udp_socket, "cw 360")
    send_control_command(udp_socket, "land")


if __name__ == "__main__":
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_stream_thread = None
    try:
        send_control_command(udp_socket, "command")
        time.sleep(0.1)
        print(f"Battery charge is {send_status_command(udp_socket, 'battery?')}%")
        send_control_command(udp_socket, "streamon")
        time.sleep(2)
        # Using VideoCapture from opencv to get frames
        vidcap = cv2.VideoCapture("udp://192.168.10.1:11111", cv2.CAP_FFMPEG)

        video_stream_thread = threading.Thread(target=drone_video, args=(vidcap, ))
        video_stream_thread.start()
        drone_prog(udp_socket)
    except Exception as exception:
        print("Exception:", exception)
    active = False
    if video_stream_thread is not None:
        video_stream_thread.join()
    # Release VideoCapture
    vidcap.release()
    cv2.destroyAllWindows()
