#! /usr/bin/env python3

import numpy as np
from queue import PriorityQueue
import math
import matplotlib.pyplot as plt
import heapq
import numpy as np
from utils import Block
from motor_control import get_odom, get_distance, odom_relocalize
from motor_control import pivot_right, pivot_left, turn_to, forward
from motor_control import Controller
from utils import Logger

logs = Logger(enable=True)


def bresenham_line(x0, y0, x1, y1):
    # Bresenham's line algorithm
    points = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return zip(*points)  # returns (rr, cc)


class Planner:
    def __init__(
        self,
        arena_size=304.8,
        resolution=1.27,
        robot_radius=4.5 * 2.54,
        object_radius=1.5 * 2.54,
    ):
        # Convert all measurements to centimeters
        self.arena_size = arena_size
        self.grid_size = int(arena_size / resolution)  # 304.8 cm / 1.27 cm = 240 cells
        self.resolution = resolution  # 1.27 cm (0.5 inch)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

        # Convert radii from inches to cells
        self.robot_radius_cells = int(robot_radius / resolution)
        self.object_radius_cells = int(object_radius / resolution)

        # Starting position (to be set)
        # self.current_pos = None
        self.current_pos = None

        # Goal position in grid coordinates
        self.goal_base = (
            int((arena_size - 2 * 2.54) / resolution),
            int((2 * 2.54) / resolution),
        )
        # self.x_decrement = int((2.5 * 2.54) / resolution)
        self.x_decrement = 0
        self.current_x_offset = 0

    def update_grid_with_objects(self, blk_list: list[Block]):
        # Mark object positions as occupied in the grid with inflation
        # Clear previous objects
        self.grid[self.grid == 1] = 0

        # Add new objects with circular footprint
        for blk in blk_list:
            x, y = int(blk.position_x / self.resolution), int(
                blk.position_y / self.resolution
            )

            # Mark circular area for object (1.5 inch radius = 3 cells)
            for i in range(
                -3, 4
            ):  # 3 cells covers 1.5 inch radius at 0.5-inch resolution
                for j in range(-3, 4):
                    if 0 <= x + i < self.grid_size and 0 <= y + j < self.grid_size:
                        distance = math.sqrt((i**2 + j**2))
                        if distance <= 3:  # 3 cells = 1.5 inch radius
                            self.grid[x + i][y + j] = 1

        # Inflate obstacles for robot size (4.5 inch radius = 9 cells)
        inflated_grid = self.grid.copy()
        occupied = np.where(self.grid == 1)
        for x, y in zip(occupied[0], occupied[1]):
            for i in range(-9, 10):  # ±9 cells covers 4.5 inch radius
                for j in range(-9, 10):
                    if 0 <= x + i < self.grid_size and 0 <= y + j < self.grid_size:
                        distance = math.sqrt((i**2 + j**2))
                        if distance <= 9:  # 9 cells = 4.5 inch radius
                            inflated_grid[x + i][y + j] = 1
        return inflated_grid

    def plot_path_on_grid(self, grid, path, start, goal):
        # Plot the grid with explored neighbors and the final path
        plt.figure(figsize=(10, 10))
        plt.imshow(grid, cmap="Greys", origin="upper")
        # Plot the path if it exists
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, c="green", label="Path")

        # Mark start and goal
        plt.scatter(start[1], start[0], c="green", s=50, label="Start")
        plt.scatter(goal[1], goal[0], c="red", s=50, label="Goal")

        plt.title("A* Path Planning with Explored Neighbors")
        plt.xlabel("X-axis (grid cells)")
        plt.ylabel("Y-axis (grid cells)")
        plt.legend()
        plt.xlim(0, self.grid_size)
        plt.ylim(0, self.grid_size)
        plt.grid(True)
        plt.show()

    def simplify_path(self, path, grid):
        def has_line_of_sight(p1, p2, grid):
            rr, cc = bresenham_line(p1[0], p1[1], p2[0], p2[1])
            return all(
                0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] and grid[r, c] == 0
                for r, c in zip(rr, cc)
            )

        if not path:
            return []

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i:
                if has_line_of_sight(path[i], path[j], grid):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
        return smoothed, len(smoothed)

    def a_star_path_planning(self, start, goal, grid):
        if start == goal:
            return [start]

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = []
        heapq.heappush(open_set, (heuristic(start, goal), 0, start))

        came_from = {}
        g_score = {start: 0}
        closed_set = set()

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                self.current_pos = path[-1]
                # Plot the path
                path, path_len = self.simplify_path(path, grid)
                # self.plot_path_on_grid(grid, path, start, goal)
                return path

            if current in closed_set:
                continue
            closed_set.add(current)

            action_set = [
                (-1, -1),
                (-1, 0),
                (-1, 1),
                (0, -1),
                (0, 1),
                (1, -1),
                (1, 0),
                (1, 1),
            ]

            for action in action_set:
                i, j = action
                neighbor = (current[0] + i, current[1] + j)
                if (
                    0 <= neighbor[0] < grid.shape[0]
                    and 0 <= neighbor[1] < grid.shape[1]
                    and grid[neighbor[0]][neighbor[1]] == 0
                ):
                    # Prevent diagonal corner cutting
                    if abs(i) == 1 and abs(j) == 1:
                        if (
                            grid[current[0] + i][current[1]] == 1
                            or grid[current[0]][current[1] + j] == 1
                        ):
                            continue

                    tentative_g = g_score[current] + np.hypot(i, j)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                        came_from[neighbor] = current

        return None

    def transport_objects(self, blk_list: list[Block], color):
        controller = Controller()
        for i, blk in enumerate(blk_list):
            target_blk = blk if color == blk.color else None
            if target_blk is None:
                continue

            # Convert object position to grid coordinates
            blk_pos = (
                int(blk.position_x / self.resolution),
                int(blk.position_y / self.resolution),
            )

            # Update grid with current objects (only remaining ones)
            inflated_grid = self.update_grid_with_objects(blk_list[i + 1 :])

            # Plan path from current position to object
            logs.logger_info(
                f"\nProcessing Object {i+1} at ({blk.position_x:.2f}cm, {blk.position_y:.2f}cm) and Grid position: {blk_pos}"
            )
            curr_odom = get_odom()
            current_pos = (
                int(curr_odom.x / self.resolution),
                int(curr_odom.y / self.resolution),
            )
            path_to_blk = self.a_star_path_planning(current_pos, blk_pos, inflated_grid)

            if not path_to_blk:
                logs.logger_info(f"Warning: No valid path found to Object {i+1}")
                continue

            # Convert path back to cm for display
            path_cm = [
                (p[0] * self.resolution, p[1] * self.resolution) for p in path_to_blk
            ]
            logs.logger_info(f"Path to object ({len(path_to_blk)} steps): {path_cm}")

            controller.open_gripper()
            controller.waypoint_follower(path_cm)
            controller.close_gripper()

            # Move robot to object (in simulation, update current position)
            self.current_pos = blk_pos
            logs.logger_info(
                f"Robot reached object at ({blk.position_x:.2f}cm, {blk.position_y:.2f}cm)"
            )

            # Calculate goal position for this object
            goal_pos = (self.goal_base[0] + self.current_x_offset, self.goal_base[1])
            self.current_x_offset -= self.x_decrement

            # Plan path from object to goal
            logs.logger_info(
                f"Planning path to goal at ({goal_pos[0] * self.resolution:.2f}cm, {goal_pos[1] * self.resolution:.2f}cm)..."
            )
            path_to_goal = self.a_star_path_planning(
                self.current_pos, goal_pos, inflated_grid
            )

            if not path_to_goal:
                logs.logger_info(f"Warning: No valid path found to goal for Object {i+1}")
                continue

            # Convert path back to cm for display
            path_cm = [
                (p[0] * self.resolution, p[1] * self.resolution) for p in path_to_goal
            ]
            logs.logger_info(f"Path to goal ({len(path_to_goal)} steps): {path_cm}")

            controller.waypoint_follower(path_cm)
            controller.open_gripper()

            # Move robot to goal
            logs.logger_info(f"Object {i+1} successfully transported to goal position")
            
            # Relocalize and remap
            turn_to(90)
            distance_x = get_distance()
            while distance_x > 20:
                forward(distance_x / 2)
                distance_x = get_distance()

            turn_to(0)
            distance_y = get_distance()
            while distance_y > 20:                
                forward(distance_y / 2)
                distance_y = get_distance()

            turn_to(90)
            distance_x = get_distance()
            while distance_x > 20:
                forward(distance_x / 2)
                distance_x = get_distance()

            odom_relocalize(distance_x, self.arena_size - distance_y)

            turn_to(-135)
            return