def get_full_drone_status(tello):
    return {
        "height": tello.get_height(),
        "(pitch, yaw, roll)": (tello.get_pitch(), tello.get_yaw(), tello.get_roll()),
        "temperature": tello.get_temperature(),
        "speed": (tello.get_speed_x(), tello.get_speed_y(), tello.get_speed_z()),
        "flight time": tello.get_flight_time(),
        "acceleration": (tello.get_acceleration_x(), tello.get_acceleration_y(), tello.get_acceleration_z()),
        "battery": f"{tello.get_battery()}%",
    }
