#! /usr/bin/env python3

import numpy as np
import logging

def normalize_angle(angle):
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def angular_difference(target, current):
    # Compute the shortest signed angular difference in degrees 
    diff = (target - current + 180) % 360 - 180
    return diff

class MovingAverage:
    def __init__(self, size):
        self.size = size
        self.buffer = [0.0] * size
        self.sum = 0.0
        self.index = 0
        self.count = 0

    def add(self, value):
        if self.count < self.size:
            # Add the new value without removing any previous value as the buffer does not contain anything yet
            self.buffer[self.index] = value
            self.sum += value
            self.count += 1
        else:
            # Remove the oldest value from the sum before replacing it
            self.sum -= self.buffer[self.index]
            self.buffer[self.index] = value
            self.sum += value

        # Move to the next index
        self.index = (self.index + 1) % self.size

    def average(self):
        if self.count == 0:
            return 0.0
        return self.sum / self.count

    def is_full(self):
        return self.count == self.size


class Odometry:
    def __init__(self, start_x, start_y, gripper_status):
        self.x, self.y, self.gripper_status = start_x, start_y, gripper_status

    def dead_reckon_update(self, dx, dy):
        self.x += dx
        self.y += dy

    def heading_update(self, theta):
        self.theta = theta

    def relocalize(self, x, y):
        self.x = x
        self.y = y

    def gripper_close(self):
        self.gripper_status = True

    def gripper_open(self):
        self.gripper_status = False

    def get_odom_status(self):
        return self.x, self.y, self.theta, self.gripper_status

class BlockMeta:
    def __init__(
        self, block_poses: dict, confidence: float, num_valid: int, odom
    ):
        self.block_poses = block_poses
        self.confidence_score = confidence
        self.num_valid = num_valid
        self.view_pose = odom


class Block:
    def __init__(self, x, y, color):
        self.position_x = x
        self.position_y = y
        self.color = color

class Logger:
    def __init__(self, enable = True):
        self.enabled = enable
        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

    def logger_info(self, msg):
        if self.enabled:
            logging.info(msg)
            
    def logger_error(self, msg):
        if self.enabled:
            logging.error(msg)

    def logger_warning(self, msg):
        if self.enabled:
            logging.warning(msg)

def transform_object_to_map_frame(robot_in_map_frame, object_in_robot_frame):
    # Transforms object coordinates from Robot frame to Map frame
    x_r, y_r, theta_r = robot_in_map_frame
    x_o, y_o = object_in_robot_frame

    # Rotation matrix
    R = np.array(
        [[np.cos(theta_r), -np.sin(theta_r)], [np.sin(theta_r), np.cos(theta_r)]]
    )

    # Transform
    obj_vec = np.array([x_o, y_o])
    transformed = np.dot(R, obj_vec) + np.array([x_r, y_r])

    return tuple(transformed)
