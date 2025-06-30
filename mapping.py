#! /usr/bin/env python3

from motor_control import pivot_left, pivot_right, turn_to
from motor_control import get_odom
from sensor_client import get_sonar_distance, get_imu_heading
from path_planner import Planner
from camera import capture_image
from vision import detect_block, detect_objects
from math import tan, radians, pi as PI
from utils  import transform_object_to_map_frame
from utils import Block, BlockMeta
from utils import Logger
import time

logs = Logger(enable=True)

class Mapping:
    def __init__(self):
        pass

    def search_objects(self):
        # Rotate incrementally in place to find objects
        blkmeta = BlockMeta(block_poses={}, confidence=0.0, num_valid=0, odom=())
        # turn_to(-135)
        # for angle in range(-130, -85, 5):
        turn_to(135)
        for angle in range(130, 85, -5):
            turn_to(angle)
            time.sleep(1.0)
            success, object_poses, confidence, valid_blocks = detect_objects()
            if not success:
                continue

            if valid_blocks > blkmeta.num_valid:
                blkmeta.num_valid = valid_blocks
                blkmeta.confidence_score = confidence
                blkmeta.view_pose = get_odom().get_odom_status()
                blkmeta.block_poses = object_poses
            elif valid_blocks == blkmeta.num_valid:
                if confidence > blkmeta.confidence_score:
                    blkmeta.num_valid = valid_blocks
                    blkmeta.confidence_score = confidence
                    blkmeta.view_pose = get_odom().get_odom_status()
                    blkmeta.block_poses = object_poses

        if blkmeta.confidence_score > 0.0:
            blocks: list[Block] = []
            logs.logger_info(f"{blkmeta.block_poses}, {blkmeta.confidence_score}, {blkmeta.view_pose}")
            x_robot, y_robot, theta_robot, _ = blkmeta.view_pose
            for block_poses in blkmeta.block_poses.items():
                if len(block_poses[1]) > 0:
                    theta_block = radians(block_poses[1][0][1])
                    x_block = block_poses[1][0][0]
                    y_block = x_block * tan(theta_block)
                    logs.logger_info(f"{x_robot}, {y_robot}, {radians(theta_robot)}, {x_block}, {y_block}")
                    x_map, y_map = transform_object_to_map_frame((x_robot, y_robot, radians(theta_robot)), (x_block, y_block))
                    blocks.append(Block(x=x_map, y=y_map, color=block_poses[0]))

            turn_to(theta_robot)
            return True, blocks

        # No block found, rotate in the opposite direction
        blkmeta = BlockMeta(block_poses={}, confidence=0.0, num_valid=0, odom=())
        # turn_to(-135)
        # for angle in range(-140, -185, -5):
        turn_to(135)
        for angle in range(140, 185, 5):
            turn_to(angle)
            time.sleep(1.0)
            success, object_poses, confidence, valid_blocks = detect_objects()
            if not success:
                continue

            if valid_blocks > blkmeta.num_valid:
                blkmeta.num_valid = valid_blocks
                blkmeta.confidence_score = confidence
                blkmeta.view_pose = get_odom().get_odom_status()
                blkmeta.block_poses = object_poses
            elif valid_blocks == blkmeta.num_valid:
                if confidence > blkmeta.confidence_score:
                    blkmeta.num_valid = valid_blocks
                    blkmeta.confidence_score = confidence
                    blkmeta.view_pose = get_odom().get_odom_status()
                    blkmeta.block_poses = object_poses

        if blkmeta.confidence_score > 0.0:
            blocks: list[Block] = []
            logs.logger_info(f"{blkmeta.block_poses}, {blkmeta.confidence_score}, {blkmeta.view_pose}")
            x_robot, y_robot, theta_robot, _ = blkmeta.view_pose
            for block_poses in blkmeta.block_poses.items():
                if len(block_poses[1]) > 0:
                    theta_block = radians(block_poses[1][0][1])
                    x_block = block_poses[1][0][0]
                    y_block = x_block * tan(theta_block)
                    logs.logger_info(f"{x_robot}, {y_robot}, {radians(theta_robot)}, {x_block}, {y_block}")
                    x_map, y_map = transform_object_to_map_frame((x_robot, y_robot, radians(theta_robot)), (x_block, y_block))
                    blocks.append(Block(x=x_map, y=y_map, color=block_poses[0]))

            turn_to(theta_robot)
            return True, blocks

        return False, []
