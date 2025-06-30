#! /usr/bin/env python3
import serial
import time
import threading
import socket
from dataclasses import dataclass
from utils import Logger


logs = Logger(enable=True)

@dataclass
class SharedMemoryIMU:
    yaw: float = 0.0
    lock: threading.Lock = threading.Lock()

class IMUServerManager:
    def __init__(self, serial_port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', baudrate=9600, host='127.0.0.1', port=9999):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.host = host
        self.port = port
        self.ser: serial.Serial = None
        self.shared = SharedMemoryIMU()
        self.stop_event = threading.Event()
        self.threads = []


    def imu_reader(self):
        while True:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                logs.logger_info("[IMUServer] Serial port opened")
                time.sleep(2)  # wait for the serial connection to stabilize
                while True:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8').strip()
                        try:
                            values = line.split('\t')
                            x = float(values[0].split(':')[1])
                            with self.shared.lock:
                                self.shared.yaw = x
                        except (IndexError, ValueError) as e:
                            logs.logger_warning(f"[IMUServer] Parse error: {e}")
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as e:
                logs.logger_error(f"[IMUServer] Serial error: {e}, retrying in 5s...")
                time.sleep(5)

    def udp_server(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        logs.logger_info(f"[IMUServer] UDP server started on {self.host}:{self.port}")
        try:
            while not self.stop_event.is_set():
                sock.settimeout(1.0)
                try:
                    data, addr = sock.recvfrom(1024)
                    if data.decode().strip() == 'GET':
                        with self.shared.lock:
                            current_yaw = self.shared.yaw
                        sock.sendto(str(current_yaw).encode(), addr)
                except socket.timeout:
                    continue
                except Exception as e:
                    logs.logger_warning(f"[IMUServer] UDP error: {e}")
        finally:
            sock.close()
            logs.logger_info("[IMUServer] UDP server closed.")

    def start(self):
        if self.is_running():
            logs.logger_warning("[IMUServer] IMU server already running.")
            return
        self.stop_event.clear()
        self.threads = [
            threading.Thread(target=self.imu_reader, daemon=True),
            threading.Thread(target=self.udp_server, daemon=True)
        ]
        for t in self.threads:
            t.start()
        logs.logger_info("[IMUServer] IMU server started.")

    def stop(self):
        if not self.is_running():
            logs.logger_warning("[IMUServer] IMU server not running.")
            return
        self.stop_event.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
            logs.logger_info("[IMUServer] Serial port closed.")
        logs.logger_info("[IMUServer] IMU server stop requested.")

    def restart(self):
        self.stop()
        time.sleep(1)
        self.start()

    def is_running(self):
        return any(t.is_alive() for t in self.threads)
