#! /usr/bin/env python3

import time
import socket
from imu_server import IMUServerManager
from sonar_server import SonarServerManager
from utils import Logger
from utils import normalize_angle, MovingAverage

logs = Logger(enable=False)

############################### SONAR CLIENT ###############################
sonar_distance = MovingAverage(size=10)


def get_distance():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(2.0)
        sock.sendto(b"GET", ("127.0.0.1", 9998))
        data, _ = sock.recvfrom(1024)
        return float(data.decode())
    except socket.timeout:
        logs.logger_warning("[SonarClient] Socket timeout, no response from server")
        return None
    finally:
        sock.close()


def sonar_spin():
    sonar_server = SonarServerManager()
    sonar_server.start()

    try:
        while True:
            if not sonar_server.is_running():
                logs.logger_info("[SonarClient] Sonar Server crashed, restarting...")
                sonar_server.restart()

            distance = get_distance()
            if distance is not None:
                logs.logger_info(f"[SonarClient] Distance: {distance:.2f}")
                global sonar_distance
                sonar_distance.add(distance)
            else:
                logs.logger_info("[SonarClient] No response from server")

            # logger_info(f"[SonarClient] Distance: {get_distance():.2f}")
            time.sleep(0.1)
    except Exception as e:
        logs.logger_error(f"[SonarClient] Stopping client... {e}")
        sonar_server.stop()


############################### SONAR CLIENT ###############################

############################### IMU CLIENT ###############################
imu_heading = 0.0


def get_yaw():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(2.0)
        sock.sendto(b"GET", ("127.0.0.1", 9999))
        data, _ = sock.recvfrom(1024)
        # Limit the heading angle between [-180, 180)
        heading = float(data.decode())
        return normalize_angle(360 - heading)
    except socket.timeout:
        logs.logger_warning("[IMUClient] Socket timeout, no response from server")
        return None
    finally:
        sock.close()


def imu_spin():
    imu_server = IMUServerManager()
    imu_server.start()

    try:
        while True:
            if not imu_server.is_running():
                logs.logger_info("[IMUClient] IMU Server crashed, restarting...")
                imu_server.restart()

            global imu_heading
            imu_heading = get_yaw()

            if imu_heading is not None:
                logs.logger_info(f"[IMUClient] Yaw: {imu_heading:.2f}")
            else:
                logs.logger_info("[IMUClient] No response from server")

            # logs.logger_info(f"[IMUClient] Yaw: {get_yaw():.2f}")
            time.sleep(0.1)
    except Exception as e:
        logs.logger_error(f"[IMUClient] Stopping client... {e}")
        imu_server.stop()


############################### IMU CLIENT ###############################

################################ SENSOR THREAD POOL ################################

# def sensor_thread_pool_exe():
#     # This function creates and starts threads for both IMU and Sonar clients (it needs to be called inside a thread as it is blocking)
#     logs.logger_info("[SensorClient] Starting IMU and Sonar spin threads…")
#     try:
#         imu_thread = threading.Thread(target=imu_spin, daemon=True, name="IMUThread")
#         sonar_thread = threading.Thread(target=sonar_spin, daemon=True, name="SonarThread")

#         imu_thread.start()
#         sonar_thread.start()

#         # Wait for threads to finish (they won't in this case)
#         imu_thread.join()
#         sonar_thread.join()
#         logs.logger_info("[SensorClient] Main thread exiting...")

#     except KeyboardInterrupt:
#         logs.logger_error("[SensorClient] Stopping threads...")
#         imu_thread.join()
#         sonar_thread.join()
#         logs.logger_info("[SensorClient] Threads stopped.")


def get_imu_heading():
    global imu_heading
    return imu_heading


def get_sonar_distance():
    global sonar_distance
    return sonar_distance.average()
