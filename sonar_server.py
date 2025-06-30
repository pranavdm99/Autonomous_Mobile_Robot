#! /usr/bin/env python3

import RPi.GPIO as gpio
import time
import threading
import socket
from dataclasses import dataclass
from utils import Logger

logs = Logger(enable=False)

TRIG = 16
ECHO = 18

@dataclass
class SharedMemorySonar:
    distance: float = 0.0
    lock: threading.Lock = threading.Lock()

class SonarServerManager:
    def __init__(self, host='127.0.0.1', port=9998):
        self.host = host
        self.port = port
        self.shared = SharedMemorySonar()
        self.stop_event = threading.Event()
        self.threads = []
        self.reset_gpio()
        self.setup_gpio()

    def setup_gpio(self):
        gpio.setmode(gpio.BOARD)
        gpio.setup(TRIG, gpio.OUT)
        gpio.setup(ECHO, gpio.IN)

        # ensure output has no value
        gpio.output(TRIG, False)
        time.sleep(0.5)
        
    def reset_gpio(self):
        gpio.cleanup()  

    def distance_reader(self):
        while True:
            try:
                logs.logger_info("[SonarServer] Starting distance measurement...")
                while True:
                    gpio.output(TRIG, True)
                    time.sleep(0.000_01)
                    gpio.output(TRIG, False)

                    # Generate echo time signal
                    pulse_start = time.time()
                    while gpio.input(ECHO) == 0:
                        pulse_start = time.time()

                    while gpio.input(ECHO) == 1:
                        pulse_end = time.time()

                    if pulse_start is None or pulse_end is None:
                        gpio.cleanup()
                        continue

                    pulse_duration = pulse_end - pulse_start

                    # Convert time to distance
                    distance = pulse_duration * 17150
                    distance = round(distance,2)

                    with self.shared.lock:
                        self.shared.distance = distance

                    time.sleep(0.01)
            except Exception as e:
                logs.logger_error(f"[SonarServer] Error in distance measurement: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def udp_server(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        logs.logger_info(f"[SonarServer] UDP server started on {self.host}:{self.port}")
        try:
            while not self.stop_event.is_set():
                sock.settimeout(1.0)
                try:
                    data, addr = sock.recvfrom(1024)
                    if data.decode().strip() == 'GET':
                        with self.shared.lock:
                            current_distance = self.shared.distance
                        sock.sendto(str(current_distance).encode(), addr)
                except socket.timeout:
                    continue
                except Exception as e:
                    logs.logger_warning(f"[SonarServer] UDP error: {e}")
        finally:
            sock.close()
            logs.logger_info("[SonarServer] UDP server closed.")

    def start(self):
        if self.is_running():
            logs.logger_warning("[SonarServer] Sonar server already running.")
            return
        self.stop_event.clear()
        self.threads = [
            threading.Thread(target=self.distance_reader, daemon=True),
            threading.Thread(target=self.udp_server, daemon=True)
        ]
        for t in self.threads:
            t.start()
        logs.logger_info("[SonarServer] Sonar server started.")

    def stop(self):
        if not self.is_running():
            logs.logger_warning("[SonarServer] Sonar server not running.")
            return
        self.reset_gpio()
        self.stop_event.set()
        logs.logger_info("[SonarServer] Sonar server stop requested.")

    def restart(self):
        self.reset_gpio()
        self.stop()
        time.sleep(1)
        self.start()

    def is_running(self):
        return any(t.is_alive() for t in self.threads)