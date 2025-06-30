#! /usr/bin/env python3

import RPi.GPIO as gpio
import time
import threading
from math import pi as PI, sin, cos, degrees, atan2, hypot
from sensor_client import get_imu_heading, get_sonar_distance
from utils import Odometry
from utils import Logger
from utils import angular_difference, normalize_angle

logs = Logger(enable=False)

# Robot parameters
WHEEL_DIAMETER_CM = 6.5
WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM
Kp_Linear = 50
Kp_Angular = 1.8

# Motor gpio pins
IN1 = 31
IN2 = 33
IN3 = 35
IN4 = 37

# Encoder input pins
BR = 12
FL = 7

# Servo Pin
SERVO_PIN = 36


odom: Odometry = None


def odom_init(start_x, start_y):
    global odom
    odom = Odometry(start_x, start_y, False)


def odom_relocalize(position_x, position_y):
    global odom
    odom.relocalize(position_x, position_y)


def odom_heading_update_loop():
    while True:
        odom.heading_update(normalize_angle(90 + get_imu_heading()))
        time.sleep(0.01)


def gpio_init():
    # Setup gpio
    gpio.setmode(gpio.BOARD)
    gpio.setup(IN1, gpio.OUT)
    gpio.setup(IN2, gpio.OUT)
    gpio.setup(IN3, gpio.OUT)
    gpio.setup(IN4, gpio.OUT)

    gpio.setup(BR, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(FL, gpio.IN, pull_up_down=gpio.PUD_UP)

    # Setup PWM (50Hz)
    global pwm_Left_Fwd
    global pwm_Left_Rev
    global pwm_Right_Fwd
    global pwm_Right_Rev
    pwm_Left_Fwd = gpio.PWM(IN1, 50)
    pwm_Left_Rev = gpio.PWM(IN2, 50)
    pwm_Right_Fwd = gpio.PWM(IN4, 50)
    pwm_Right_Rev = gpio.PWM(IN3, 50)

    # Initialize PWM
    pwm_Left_Fwd.start(0)
    pwm_Left_Rev.start(0)
    pwm_Right_Fwd.start(0)
    pwm_Right_Rev.start(0)

    # Set up servo PWM
    gpio.setup(SERVO_PIN, gpio.OUT)
    global servo_pwm
    servo_pwm = gpio.PWM(SERVO_PIN, 50)  # 50Hz PWM for servo
    servo_pwm.start(0)


def get_odom():
    global odom
    return odom


def get_distance():
    return get_sonar_distance()


def set_motor_speeds(left_speed, right_speed):
    # Left motors
    if left_speed >= 0:
        pwm_Left_Fwd.ChangeDutyCycle(abs(left_speed))
        pwm_Left_Rev.ChangeDutyCycle(0.0)
    else:
        pwm_Left_Rev.ChangeDutyCycle(abs(left_speed))
        pwm_Left_Fwd.ChangeDutyCycle(0.0)

    # Right motors
    if right_speed >= 0:
        pwm_Right_Fwd.ChangeDutyCycle(abs(right_speed))
        pwm_Right_Rev.ChangeDutyCycle(0.0)
    else:
        pwm_Right_Rev.ChangeDutyCycle(abs(right_speed))
        pwm_Right_Fwd.ChangeDutyCycle(0.0)


def forward(distance_cm, tf=10.0):
    global odom
    ticks_per_rev = 20
    target_ticks = int((distance_cm / WHEEL_CIRCUMFERENCE_CM) * ticks_per_rev)

    base_speed = 40
    counterFL = counterBR = 0
    buttonFL = buttonBR = 0
    last_ticks = 0.0

    start_time = time.perf_counter()
    action_complete = False
    while start_time + tf >= time.perf_counter():
        if gpio.input(BR) != buttonBR:
            buttonBR = gpio.input(BR)
            counterBR += 1
        if gpio.input(FL) != buttonFL:
            buttonFL = gpio.input(FL)
            counterFL += 1

        current_ticks = (counterFL + counterBR) / 2.0
        delta_ticks = current_ticks - last_ticks
        last_ticks = current_ticks

        delta_distance = (delta_ticks / ticks_per_rev) * WHEEL_CIRCUMFERENCE_CM
        _, _, theta, _ = odom.get_odom_status()
        dx = delta_distance * cos(theta)
        dy = delta_distance * sin(theta)
        if odom is not None:
            odom.dead_reckon_update(dx, dy)

        # Check if the robot is about to collide:
        distance = get_sonar_distance()
        if distance < 15.0:  # Minimum distance to collide
            action_complete = False
            break

        error = counterFL - counterBR
        correction = Kp_Linear * error
        set_motor_speeds(
            max(0, min(100, base_speed - correction)),
            max(0, min(100, base_speed + correction)),
        )

        if counterFL >= target_ticks or counterBR >= target_ticks:
            action_complete = True
            break

        time.sleep(0.01)

    return action_complete


def pivot_left(target_angle, tf=3.0):
    logs.logger_info("Pivoting left")
    global odom
    _, _, initial_angle, _ = odom.get_odom_status()
    if initial_angle is None:
        return False

    start_time = time.perf_counter()
    while (start_time + tf) >= time.perf_counter():
        _, _, theta, _ = odom.get_odom_status()
        # completed_angle = initial_angle - theta
        completed_angle = angular_difference(theta, initial_angle)
        error = target_angle - completed_angle
        speed = max(60, min(100, Kp_Angular * error))
        logs.logger_info(
            f"Pivoting left at {speed}. Completed {completed_angle} degrees with respect to the initial {initial_angle} angle"
        )

        if completed_angle >= target_angle:
            logs.logger_info(f"Reached Target: {theta}")
            break

        set_motor_speeds(-speed, speed)
        logs.logger_info(f"{theta}, {completed_angle}, {error}")
        time.sleep(0.01)

    set_motor_speeds(0, 0)
    return True


def pivot_right(target_angle, tf=3.0):
    logs.logger_info("Pivoting right")
    _, _, initial_angle, _ = odom.get_odom_status()
    if initial_angle is None:
        return False

    start_time = time.perf_counter()
    while (start_time + tf) >= time.perf_counter():
        _, _, theta, _ = odom.get_odom_status()        
        # completed_angle = theta - initial_angle
        completed_angle = -angular_difference(theta, initial_angle)
        error = target_angle - completed_angle
        speed = max(60, min(100, Kp_Angular * error))
        logs.logger_info(
            f"Pivoting right at {speed}. Completed {completed_angle} degrees with respect to the initial {initial_angle} angle"
        )

        if completed_angle >= target_angle:
            logs.logger_info(f"Reached Target: {theta:.2f}")
            break

        set_motor_speeds(speed, -speed)
        logs.logger_info(f"{theta}, {completed_angle}, {error}")
        time.sleep(0.01)

    set_motor_speeds(0, 0)
    return True


def open_gripper():
    duty_cycle = 12.5
    global servo_pwm
    servo_pwm.ChangeDutyCycle(duty_cycle)
    odom.gripper_open()


def close_gripper():
    duty_cycle = 8.5
    global servo_pwm
    servo_pwm.ChangeDutyCycle(duty_cycle)
    odom.gripper_close()


def gpio_cleanup():
    set_motor_speeds(0, 0)
    gpio.output(IN1, False)
    gpio.output(IN2, False)
    gpio.output(IN3, False)
    gpio.output(IN4, False)
    gpio.cleanup()


def stop():
    set_motor_speeds(0, 0)


def gameover():
    gpio_cleanup()

def turn_to(target_heading_deg, tf=3.0):
    # Rotate in place to an absolute heading (in degrees).
    # Positive angles are CCW from map-frame zero.
    # Get current heading
    _, _, current_heading_deg, _ = odom.get_odom_status()
    if current_heading_deg is None:
        logs.logger_info("Odometry heading not available")
        return False

    # Compute minimal signed angle difference
    angle_diff = normalize_angle(target_heading_deg - current_heading_deg)
    logs.logger_info(f"Heading {current_heading_deg:.1f}, {target_heading_deg:.1f}, diff {angle_diff:.1f}")

    # Choose pivot direction
    if angle_diff > 0:
        return pivot_left(angle_diff, tf)
    else:
        return pivot_right(abs(angle_diff), tf)


class Controller:
    def open_gripper(self):
        open_gripper()

    def close_gripper(self):
        close_gripper()

    def waypoint_follower(self, waypoints):
        for target_x, target_y in waypoints:
            x, y, theta, _ = get_odom().get_odom_status()

            dx = target_x - x
            dy = target_y - y
            target_angle = degrees(atan2(dy, dx))
            angle_diff = normalize_angle(target_angle - theta)

            # Rotate in place
            if angle_diff > 0:
                pivot = pivot_left
            else:
                pivot = pivot_right

            pivot(abs(angle_diff))

            # Move forward to reach the goal
            distance = hypot(dx, dy)
            forward(distance)

        stop()

