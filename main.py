#! /usr/bin/env python3

import sys
import threading
from mapping import Mapping
from path_planner import Planner
from utils import Logger
from sensor_client import imu_spin, sonar_spin
from motor_control import forward, pivot_left, pivot_right, turn_to
from motor_control import odom_init, odom_heading_update_loop, gpio_init, get_distance
import time
from math import pi as PI
logs = Logger(enable=True)


def challenge():
    # Set starting position (50cm, 50cm)    
    gpio_init()

    time.sleep(2.0)

    distance_x = get_distance()
    while distance_x > 50:
        logs.logger_info(f"Remaining distance {distance_x}, moving forward {distance_x / 4}")
        forward(distance_x / 4)
        distance_x = get_distance()
    forward(5*12*2.54)
    turn_to(0)

    logs.logger_info("[Main] Reached the construction zone")
    time.sleep(2.0)

    map = Mapping()
    success, blocks = map.search_objects()
    color = 0 
    key_color = {"red", "green", "blue"}
    planner = Planner(arena_size=7*12*2.54)
    while success:
        success, blocks = map.search_objects()
        if success:
            logs.logger_info(
                "[Main] Mapping successul, proceeding with block transportation"
            )

            planner.transport_objects(blocks, key_color[color])
            color = (color + 1) % 3
        else:
            logs.logger_info("[Main] Failed mapping, exit")
            exit(1)


def start_thread_pool_exe():
    if len(sys.argv) >= 3:
        start_x, start_y = float(sys.argv[1]), float(sys.argv[2])
    else:
        start_x, start_y = 30.48, 15.24  # By default, start at (12 inch, 6 inch)

    odom_init(start_x, start_y)

    # Create threads to execute the challenge and the sensor clients for both IMU and Sonar
    try:
        logs.logger_info("[Main] Starting IMU and Sonar spin threads…")

        imu_thread = threading.Thread(target=imu_spin, daemon=True, name="IMUThread")

        sonar_thread = threading.Thread(
            target=sonar_spin, daemon=True, name="SonarThread"
        )

        odom_thread = threading.Thread(
            target=odom_heading_update_loop, daemon=True, name="OdomThread"
        )

        challenge_thread = threading.Thread(
            target=challenge, daemon=True, name="Challenge"
        )

        # Start the threads
        odom_thread.start()
        imu_thread.start()
        sonar_thread.start()
        challenge_thread.start()

        # Wait for threads to finish (they won't in this case)
        odom_thread.join()
        imu_thread.join()
        sonar_thread.join()
        challenge_thread.join()
        logs.logger_info("[Main] Main thread exiting...")

    except KeyboardInterrupt:
        logs.logger_error("[Main] Stopping threads...")
        odom_thread.join()
        imu_thread.join()
        sonar_thread.join()
        challenge_thread.join()
        logs.logger_info("[Main] Threads stopped.")

    finally:
        logs.logger_error("[Main] Challenge completed...")
        odom_thread.join()
        imu_thread.join()
        sonar_thread.join()
        challenge_thread.join()
        logs.logger_info("[Main] Threads stopped.")


def main():
    start_thread_pool_exe()


if __name__ == "__main__":
    main()
