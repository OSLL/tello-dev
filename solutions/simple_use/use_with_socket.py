import cv2
import time
import threading
import socket


class SimpleSocketDemo():

    def __init__(self):
        self.IP = "192.168.10.1"
        self.CONTROL_PORT = 8889
        self.STATUS_PORT = 8890
        self.RETRIES = 5
        self.active = True
        self.udp_socket = None
        self.video_stream_thread = None

    def drone_video(self, backreader):
        try:
            while self.active:
                # Read frame
                ret, cur_frame = backreader.read()
                if ret is None:
                    continue
                cv2.imshow("Drone stream", cur_frame)
                cv2.waitKey(1)
        except Exception as exception:
            print("Exception in drone_video:", exception)

    def send_control_command(self, command: str):
        if self.udp_socket is None:
            return False
        ok = False
        attempts = self.RETRIES
        while not ok or attempts > 0:
            self.udp_socket.sendto(bytes(command, "utf-8"), (self.IP, self.CONTROL_PORT))
            data, addr = self.udp_socket.recvfrom(1024, socket.MSG_DONTWAIT)
            if data.decode("utf-8") == "ok":
                ok = True
            else:
                attempts -= 1
        if attempts == 0:
            return False
        else:
            return True

    def send_status_command(self, command: str):
        if self.udp_socket is None:
            return ""
        self.udp_socket.sendto(bytes(command, "utf-8"), (self.IP, self.CONTROL_PORT))
        data, addr = self.udp_socket.recvfrom(1024, socket.MSG_DONTWAIT)
        return data.decode("utf-8").replace("\r\n", "")

    def drone_prog(self):
        if not self.send_control_command("takeoff"):
            print("Cannot exec command takeoff")
            return
        if not self.send_control_command("forward 100"):
            print("Cannot exec command forward 100")
            return
        time.sleep(1)
        if not self.send_control_command("up 100"):
            print("Cannot exec command up 100")
            return
        time.sleep(1)
        if not self.send_control_command("cw 360"):
            print("Cannot exec command cw 360")
            return
        if not self.send_control_command("land"):
            print("Cannot exec command land")
            return

    def main(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_stream_thread = None
        vidcap = None
        try:
            self.send_control_command("command")
            time.sleep(0.1)
            print(f"Battery charge is {send_status_command('battery?')}%")
            self.send_control_command("streamon")
            time.sleep(2)
            # Using VideoCapture from opencv to get frames
            vidcap = cv2.VideoCapture("udp://192.168.10.1:11111", cv2.CAP_FFMPEG)

            self.video_stream_thread = threading.Thread(target=drone_video, args=(vidcap, ))
            self.video_stream_thread.start()
            self.drone_prog()
        except Exception as exception:
            print("Exception:", exception)
        self.active = False
        if self.video_stream_thread is not None:
            self.video_stream_thread.join()
        # Release VideoCapture
        if vidcap is not None:
            vidcap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.active = False
        ok = False
        while not ok:
            ok = self.send_control_command("land")
