#! /usr/bin/env python3

import cv2
import numpy as np
from math import asin, sin, cos, atan, degrees, radians, exp
import time
from utils import MovingAverage
from camera import capture_image
from utils import Logger

logs = Logger(enable=False)

lower_green = np.array([42, 32, 140])
upper_green = np.array([56, 211, 255])
lower_red = np.array([0, 151, 109])
upper_red = np.array([5, 255, 255])
lower_blue = np.array([71, 70, 103])
upper_blue = np.array([116, 240, 255])

hsv_filter = {
    "red": [lower_red, upper_red],
    "green": [lower_green, upper_green],
    "blue": [lower_blue, upper_blue],
}
bgr_colors = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}

WIDTH, HEIGHT = 1280, 960

# Camera Calibration Parameters
# camera_matrix = np.array([[1393.13, 0, 331.68],
#                             [0, 1377.55, 146.61],
#                             [0, 0, 1]])
# dist_coeffs = np.array([0.0362, 4.7928, -0.00857, -0.0045, -52.8583])

camera_matrix = np.array(
    [[1014.39401, 0, 625.087547], [0, 1020.29423, 453.042810], [0, 0, 1]]
)
distortion_coeffs = np.array(
    [0.320273044, -0.781386009, -0.00291847070, 0.000551282239, 0.991841065]
)

fx = camera_matrix[0, 0]  # focal length in pixels
fy = camera_matrix[1, 1]
block_height_cm = 5.715
hfov = degrees(2 * atan((WIDTH / 2) / fx))
vfov = degrees(2 * atan((HEIGHT / 2) / fy))


def detect_block(key, samples=None, timeout=3.0):
    # Filters
    lowerf, upperf, color = hsv_filter[key][0], hsv_filter[key][1], bgr_colors[key]

    distances, angles = MovingAverage(size=10), MovingAverage(size=10)
    count = 0
    start_time = time.perf_counter()
    frame = None
    while True:
        if samples is not None:
            if count > samples:
                return True, distances.average(), angles.average(), frame
            if (time.perf_counter() - start_time) > timeout:
                # Timed out, return false
                return False, distances.average(), angles.average(), frame

        frame = capture_image()
        h, w = frame.shape[0], frame.shape[1]

        # Undistort the image
        # undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the mask based on the HSV filter
        mask = cv2.inRange(hsv, lowerf, upperf)

        # Mask out the top 1/4th and the bottom 1/4th of the image
        mask[: h // 4, :] = mask[3 * h // 4 :, :] = 0

        image_center_x, image_center_y = frame.shape[1] // 2, frame.shape[0] // 2
        cv2.line(
            frame,
            (image_center_x - 100, image_center_y),
            (image_center_x + 100, image_center_y),
            (0, 0, 0),
            1,
        )
        cv2.line(
            frame,
            (image_center_x, image_center_y - 100),
            (image_center_x, image_center_y + 100),
            (0, 0, 0),
            1,
        )

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Sort the contours in descending order of the area
            # sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
            largest_contours = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(largest_contours)
            if area > 50:
                # Draw a bounding box around the contour
                x, y, w, h = cv2.boundingRect(largest_contours)

                # Estimate angle
                pixel_width, pixel_height = w, h
                aspect_ratio = pixel_width / pixel_height
                logs.logger_info(f"Aspect Ratio: {aspect_ratio}")
                if aspect_ratio < 0.66 or aspect_ratio > 0.94:
                    # Aspect ratio is not in range, so we do not estimate the angle, but continue looking for the object
                    logs.logger_info(f"Unknown Aspect Ratio: {aspect_ratio}")
                    cv2.imshow("Angle Estimation", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    # Aspect ratio is valid
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    block_center_x, block_center_y = x + w // 2, y + h // 2
                    pixel_offset_x = image_center_x - block_center_x
                    degrees_per_pixel = hfov / WIDTH
                    angle_offset_deg = pixel_offset_x * degrees_per_pixel

                    # Using moving average on the estimated average to soften fluctuations
                    angles.add(angle_offset_deg)

                    # The object may be at an angle, so determine the angle using the known aspect ratio (which is valid)
                    theta = abs(0.5 * asin(2.25 * aspect_ratio**2 - 1))
                    # The actual width of the image is based on the angle the object makes with respect to the camera (basic trig)
                    block_width_cm = 3.81 * (sin(theta) + cos(theta))
                    # The distance is the ratio of the actual image width/height and pixel width scaled by the focal length
                    distance_x = (fx * block_width_cm) / pixel_width
                    distance_y = (fy * block_height_cm) / pixel_height

                    # Using weighted average to estimate the distance
                    expected_distance = distance_x * 1.0 + distance_y * 0.0

                    # Using moving average on the estimated average to soften fluctuations
                    distances.add(expected_distance)
                    count += 1
                    if distances.is_full() and angles.is_full():
                        cv2.putText(
                            frame,
                            f"{distances.average():.2f} cm",
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2,
                        )
                        cv2.putText(
                            frame,
                            f"{angles.average():.2f} deg",
                            (x, y + h + 16),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2,
                        )
            else:
                # Object is too small, skipping
                cv2.imshow("Angle Estimation", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        cv2.imshow("Angle Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        logs.logger_info(f"Elapsed Time: {time.perf_counter() - start_time}")

    if count > 10:
        return True, distances.average(), angles.average(), frame

    return False, distances.average(), angles.average(), frame


def detect_objects(max_expected_blocks=9):
    object_pose = {"red": [], "green": [], "blue": []}
    frame: cv2.typing.MatLike = capture_image()
    frame = cv2.undistort(frame, camera_matrix, distortion_coeffs)
    image_center_x, image_center_y = frame.shape[1] // 2, frame.shape[0] // 2

    aspect_score_sum = 0
    solidity_sum = 0
    valid_blocks = 0

    for key, value in bgr_colors.items():
        lowerf, upperf, color = hsv_filter[key][0], hsv_filter[key][1], value
        height, width = frame.shape[0], frame.shape[1]

        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lowerf, upperf)
        mask[: height // 4, :] = mask[3 * height // 4 : height, :] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in sorted_contours:
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                logs.logger_info(f"Aspect Ratio: {aspect_ratio}")

                if 0.6 <= aspect_ratio <= 0.94:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    block_center_x = x + w // 2
                    pixel_offset_x = image_center_x - block_center_x
                    degrees_per_pixel = hfov / WIDTH
                    angle_offset_deg = pixel_offset_x * degrees_per_pixel

                    theta = abs(0.5 * asin(2.25 * aspect_ratio**2 - 1))
                    block_width_cm = 3.81 * (sin(theta) + cos(theta))
                    distance_x = (fx * block_width_cm) / w
                    distance_y = (fy * block_height_cm) / h
                    expected_distance = distance_x * 1.0 + distance_y * 0.0

                    object_pose[key].append((expected_distance, angle_offset_deg))

                    # Accumulate confidence metrics

                    # Udpate aspect Score sum: How much the aspect ratio deviates with respect to the expected value
                    aspect_score_sum += exp(-4.0 * abs(aspect_ratio - 0.8))

                    # Update solidity: How well the object fills its bounding box
                    bounding_box_area = w * h
                    solidity = (
                        area / float(bounding_box_area) if bounding_box_area > 0 else 0
                    )
                    solidity_sum += min(solidity, 1.0)

                    # Update number of blocks detected
                    valid_blocks += 1
                else:
                    logs.logger_info(f"Unknown Aspect Ratio: {aspect_ratio}")
            else:
                continue

    if valid_blocks == 0:
        return False, object_pose, 0.0, 0

    # Normalize and compute composite confidence score
    confidence = (
        0.5 * (valid_blocks / max_expected_blocks)
        + 0.25 * (aspect_score_sum / valid_blocks)
        + 0.25 * (solidity_sum / valid_blocks)
    )

    return True, object_pose, round(confidence, 3), valid_blocks


# def detect_objects():
#     object_pose = {"red":list(tuple()), "green":list(tuple()), "blue":list(tuple())}
#     frame : cv2.typing.MatLike = capture_image()
#     frame = cv2.undistort(frame, camera_matrix, distortion_coeffs)
#     for key, value in bgr_colors.items():
#         # Filters
#         lowerf, upperf, color = hsv_filter[key][0], hsv_filter[key][1], value

#         height, width = frame.shape[0], frame.shape[1]

#         # Convert to HSV and get the mask based on the HSV filter
#         mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lowerf, upperf)

#         # Mask out the top 1/4th and the bottom 1/4th of the image
#         mask[: height // 4, :] = mask[3 * height // 4 : height, :] = 0

#         image_center_x, image_center_y = frame.shape[1] // 2, frame.shape[0] // 2
#         cv2.line(
#             frame,
#             (image_center_x - 100, image_center_y),
#             (image_center_x + 100, image_center_y),
#             (0, 0, 0),
#             1,
#         )
#         cv2.line(
#             frame,
#             (image_center_x, image_center_y - 100),
#             (image_center_x, image_center_y + 100),
#             (0, 0, 0),
#             1,
#         )

#         # Find contours
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours:
#             continue

#         # Sort the contours in descending order of the area
#         sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
#         for contour in sorted_contours:
#             area = cv2.contourArea(contour)
#             if area > 100:
#                 # Draw a bounding box around the contour
#                 x, y, w, h = cv2.boundingRect(contour)

#                 # Estimate angle
#                 pixel_width, pixel_height = w, h
#                 aspect_ratio = pixel_width / pixel_height
#                 logs.logger_info(f"Aspect Ratio: {aspect_ratio}")
#                 if 0.66 <= aspect_ratio <= 0.94:
#                     # Aspect ratio is valid
#                     cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
#                     block_center_x, block_center_y = x + w // 2, y + h // 2
#                     pixel_offset_x = block_center_x - image_center_x
#                     degrees_per_pixel = hfov / WIDTH
#                     angle_offset_deg = pixel_offset_x * degrees_per_pixel

#                     # The object may be at an angle, so determine the angle using the known aspect ratio (which is valid)
#                     theta = abs(0.5 * asin(2.25 * aspect_ratio**2 - 1))
#                     # The actual width of the image is based on the angle the object makes with respect to the camera (basic trig)
#                     block_width_cm = 3.81 * (sin(theta) + cos(theta))
#                     # The distance is the ratio of the actual image width/height and pixel width scaled by the focal length
#                     distance_x = (fx * block_width_cm) / pixel_width
#                     distance_y = (fy * block_height_cm) / pixel_height

#                     # Using weighted average to estimate the distance
#                     expected_distance = distance_x * 1.0 + distance_y * 0.0

#                     # Append the distance and angle of the block to the list
#                     object_pose[key].append((expected_distance, angle_offset_deg))
#                 else:
#                     # Aspect ratio is not in range, so we do not estimate the angle, but continue looking for the object
#                     logs.logger_info(f"Unknown Aspect Ratio: {aspect_ratio}")
#                     cv2.imshow("Angle Estimation", frame)
#                     if cv2.waitKey(1) & 0xFF == ord("q"):
#                         break
#             else:
#                 # Object is too small, skipping
#                 cv2.imshow("Angle Estimation", frame)
#                 if cv2.waitKey(1) & 0xFF == ord("q"):
#                     break

#         return False, object_pose, confidence
