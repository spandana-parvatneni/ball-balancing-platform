# Ported and adapted from giusenso/Ball-Balancing-PID-System (MIT License)
# Original: https://github.com/giusenso/Ball-Balancing-PID-System

"""
ball_tracker.py
Port of ball_tracker.cpp — HSV detection, morphological ops, contour tracking, overlay drawing.

trackFilteredObject() equivalent: track_filtered_object()
morphOps() equivalent:            morph_ops()
drawObjectV2() equivalent:        draw_overlay()
"""

import cv2
import numpy as np
from ball_physic import update_ball

MIN_OBJECT_AREA = 14 * 14   # minimum contour area to be considered a ball
MAX_NUM_OBJECTS = 3          # if more contours than this, frame is too noisy


def morph_ops(thresh):
    """
    Erode twice then dilate twice.
    Removes noise specks and fills gaps in ball blob.
    5x5 rectangular kernel — same as original.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    thresh = cv2.erode(thresh,  kernel, iterations=2)
    thresh = cv2.dilate(thresh, kernel, iterations=2)
    return thresh


def track_filtered_object(ball, threshold, frame_w, frame_h, control_area):
    """
    Find ball in HSV threshold mask using contour moments method.
    Updates ball object in place — sets ball.detected and calls update_ball().

    Offsets centroid from ROI-local coordinates back to full frame coordinates.
    """
    max_object_area = (frame_h * frame_w) // 5

    contours, hierarchy = cv2.findContours(
        threshold.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if hierarchy is None or len(hierarchy) == 0:
        ball.detected = False
        return

    num_objects = len(hierarchy[0])

    # too many contours = noise; don't trust detection
    if num_objects > MAX_NUM_OBJECTS:
        ball.detected = False
        return

    best_area = 0
    best_cx = None
    best_cy = None

    for i in range(num_objects):
        m    = cv2.moments(contours[i])
        area = m['m00']

        if MIN_OBJECT_AREA < area < max_object_area and area > best_area:
            best_area = area
            # offset centroid back to full-frame coordinates
            # (detection runs on a centre-cropped ROI)
            offset_x = (frame_w - control_area) // 2
            offset_y = (frame_h - control_area) // 2
            best_cx  = int(m['m10'] / area) + offset_x
            best_cy  = int(m['m01'] / area) + offset_y

    if best_cx is not None:
        if ball.detected:
            update_ball(ball, best_cx, best_cy)
        else:
            # first detection — initialise all history to current position
            ball.x          = [best_cx] * 8
            ball.y          = [best_cy] * 8
            ball.dx         = [0] * 8
            ball.dy         = [0] * 8
            ball.smooth_dx  = 0
            ball.smooth_dy  = 0
            ball.detected   = True
    else:
        ball.detected = False


def draw_overlay(frame, ball, setpoint_x, setpoint_y,
                 control_area, pid_x=None, pid_y=None):
    """
    Draw debug overlay on camera frame:
      - Control area bounding box
      - Setpoint marker
      - Ball position box and crosshair
      - Velocity arrow
      - Previous position trail (8 frames)
      - Text: coordinates, PID output, status
    """
    h, w = frame.shape[:2]

    # control area box
    cv2.rectangle(frame,
                  (setpoint_x - control_area // 2, setpoint_y - control_area // 2),
                  (setpoint_x + control_area // 2, setpoint_y + control_area // 2),
                  (212, 214, 44), 2)

    # setpoint dot
    cv2.circle(frame, (setpoint_x, setpoint_y), 4, (212, 214, 44), -1)

    if ball.detected:
        bx, by = ball.x[0], ball.y[0]

        # ball bounding box
        cv2.rectangle(frame, (bx - 28, by - 28), (bx + 28, by + 28),
                      (24, 244, 0), 1)

        # crosshair lines
        cv2.line(frame, (bx, 0),    (bx, h), (205, 45, 0), 1)
        cv2.line(frame, (0, by),    (w, by), (205, 45, 0), 1)

        # velocity arrow
        cv2.arrowedLine(frame,
                        (bx, by),
                        (bx + int(ball.smooth_dx), by + int(ball.smooth_dy)),
                        (0, 0, 251), 2, tipLength=0.4)

        # previous position trail
        for i in range(1, 8):
            cv2.circle(frame, (ball.x[i], ball.y[i]), 2, (0, 165, 254), -1)

        # text labels
        cv2.putText(frame, f"x:{bx}  y:{by}",
                    (bx + 32, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (205, 45, 0), 1)
        cv2.putText(frame, "BALL FOUND",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (24, 244, 0), 2)

        if pid_x is not None and pid_y is not None:
            cv2.putText(frame,
                        f"Xout:{pid_x.output[0]:.0f}  Yout:{pid_y.output[0]:.0f}",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        cv2.putText(frame, "BALL NOT FOUND",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 251), 2)
