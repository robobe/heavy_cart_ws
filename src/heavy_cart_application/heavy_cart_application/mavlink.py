#!/usr/bin/env python3

import time
from pymavlink import mavutil
from threading import Thread
import signal

# MAVLink connection (e.g., UDP or serial)
master = mavutil.mavlink_connection('udpout:localhost:14550')  # Broadcast to localhost on port 14550

class Sender():
    def __init__(self) -> None:
        ...

    def run(self):
        self.t = Thread(target=self.__run, daemon=True, name="mav_sender")
        self.t.start()


    def __run(self):
        while True:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GROUND_ROVER,  # MAVLink type, GCS (ground control station)
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,  # Autopilot type
                mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                0, 0, 0  # Random values for system_status, base_mode, custom_mode, etc.
            )
            
            print("Heartbeat sent")
            time.sleep(1)  # Wait for 1 second before sending the next heartbeat

class Receiver():
    def __init__(self) -> None:
        self.__handlers = {}

    def run(self):
        self.t = Thread(target=self.__run, daemon=True, name="mav_sender")
        self.t.start()


    def register_handler(self, msg_type, handler):
        self.__handlers[msg_type] = handler

    def __run(self):
        while True:
            msg = master.recv_match(blocking=True)  # Non-blocking receive
            if msg:
                msg_type = msg.get_type()
                # if msg_type == "RC_CHANNELS_OVERRIDE":
                if msg_type in self.__handlers:
                    self.__handlers[msg_type](msg)

            # time.sleep(0.01)

if __name__ == "__main__":
    sender = Sender()
    sender.run()

    receiver = Receiver()
    receiver.run()

    signal.pause()
