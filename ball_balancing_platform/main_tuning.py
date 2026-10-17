# """
# main.py
# Entry point — three modes selectable via command line argument.

# Usage:
#   python main.py            # standard mode (no overlay, clean loop)
#   python main.py debug      # standard + overlay + mask window
#   python main.py hsv        # HSV tuner only (saves values to config.json)

# Architecture:
#   Laptop  : camera → ball detection → physics filter → PID → serial
#   Arduino : receives two angles over serial → drives servoX and servoY

# Serial protocol:
#   Python sends:  "90,85\n"   (X angle, Y angle as comma-separated integers)
#   Arduino reads: parses comma, writes each to servo

# Coordinate system:
#   (0,0) = top-left of frame
#   setpoint = frame centre (320, 240) at 640x480
#   error > 0 means ball is below/right of setpoint
# """

# import cv2
# import numpy as np
# import serial
# import time
# import json
# import sys
# import csv

# from ball_physic  import create_ball, print_ball
# from ball_tracker import track_filtered_object, morph_ops, draw_overlay
# from pid          import PID, pid_compute, change_setpoint, print_pid, saturation_filter


# # ── Load config ───────────────────────────────────────────────────────────────
# with open('config.json') as f:
#     cfg = json.load(f)

# PID_CFG  = cfg['pid']
# HSV_CFG  = cfg['hsv']
# SRV_CFG  = cfg['servo']
# CAM_IDX  = cfg['camera_index']
# PORT     = cfg['serial_port']
# BAUD     = cfg['baud_rate']

# FRAME_W      = 640
# FRAME_H      = 480
# CONTROL_AREA = 400          # px — only the centre 400x400 region is processed
# SETPOINT_X   = FRAME_W // 2
# SETPOINT_Y   = FRAME_H // 2
# SERVO_CENTER = SRV_CFG['center']    # 90
# SERVO_LIMIT  = SRV_CFG['limit']     # 20 → range is 70–110

# MODE = sys.argv[1] if len(sys.argv) > 1 else 'standard'


# # ── Serial ────────────────────────────────────────────────────────────────────
# print(f"\nOpening serial on {PORT} at {BAUD} baud...")
# arduino = serial.Serial(PORT, BAUD, timeout=1)
# time.sleep(2)                       # wait for Arduino reset
# arduino.reset_input_buffer()
# print("Serial ready.\n")


# def send_angles(ax, ay):
#     """Clamp and send two servo angles to Arduino over serial."""
#     ax = int(saturation_filter(ax, SERVO_CENTER - SERVO_LIMIT, SERVO_CENTER + SERVO_LIMIT))
#     ay = int(saturation_filter(ay, SERVO_CENTER - SERVO_LIMIT, SERVO_CENTER + SERVO_LIMIT))
#     arduino.write(f"{ax},{ay}\n".encode())
#     arduino.flush()


# # ── Camera ────────────────────────────────────────────────────────────────────
# print(f"Opening camera index {CAM_IDX}...")
# cap = cv2.VideoCapture(CAM_IDX)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
# cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)       # reduce latency
# if not cap.isOpened():
#     print("ERROR: Could not open camera. Check camera_index in config.json.")
#     arduino.close()
#     exit(1)
# print("Camera ready.\n")


# # ─────────────────────────────────────────────────────────────────────────────
# # HSV TUNER MODE
# # ─────────────────────────────────────────────────────────────────────────────
# if MODE == 'hsv':
#     print("HSV Tuner mode. Adjust sliders until only the ball is white in the mask.")
#     print("Press S to save and exit. Press Q to exit without saving.\n")

#     def _nothing(x): pass

#     cv2.namedWindow('HSV Tuner')
#     cv2.createTrackbar('H min', 'HSV Tuner', HSV_CFG['min'][0], 179, _nothing)
#     cv2.createTrackbar('H max', 'HSV Tuner', HSV_CFG['max'][0], 179, _nothing)
#     cv2.createTrackbar('S min', 'HSV Tuner', HSV_CFG['min'][1], 255, _nothing)
#     cv2.createTrackbar('S max', 'HSV Tuner', HSV_CFG['max'][1], 255, _nothing)
#     cv2.createTrackbar('V min', 'HSV Tuner', HSV_CFG['min'][2], 255, _nothing)
#     cv2.createTrackbar('V max', 'HSV Tuner', HSV_CFG['max'][2], 255, _nothing)

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             continue
#         hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#         lo   = np.array([cv2.getTrackbarPos('H min', 'HSV Tuner'),
#                          cv2.getTrackbarPos('S min', 'HSV Tuner'),
#                          cv2.getTrackbarPos('V min', 'HSV Tuner')])
#         hi   = np.array([cv2.getTrackbarPos('H max', 'HSV Tuner'),
#                          cv2.getTrackbarPos('S max', 'HSV Tuner'),
#                          cv2.getTrackbarPos('V max', 'HSV Tuner')])
#         mask = cv2.inRange(hsv, lo, hi)
#         cv2.imshow('HSV Tuner', frame)
#         cv2.imshow('Mask',      mask)

#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('s'):
#             cfg['hsv']['min'] = lo.tolist()
#             cfg['hsv']['max'] = hi.tolist()
#             with open('config.json', 'w') as f:
#                 json.dump(cfg, f, indent=2)
#             print(f"Saved HSV min={lo.tolist()} max={hi.tolist()} to config.json")
#             break
#         elif key == ord('q'):
#             print("Exiting without saving.")
#             break

#     cap.release()
#     cv2.destroyAllWindows()
#     arduino.close()
#     exit(0)


# # ─────────────────────────────────────────────────────────────────────────────
# # STANDARD / DEBUG MODE
# # ─────────────────────────────────────────────────────────────────────────────

# # ── PID objects ───────────────────────────────────────────────────────────────
# print("Creating PID controllers...")
# pid_x = PID(
#     kp=PID_CFG['x']['kp'], ki=PID_CFG['x']['ki'], kd=PID_CFG['x']['kd'],
#     setpoint=SETPOINT_X,
#     inverted=True,                              # flip if ball moves wrong way on X
#     servo_min=SERVO_CENTER - SERVO_LIMIT,
#     servo_max=SERVO_CENTER + SERVO_LIMIT,
#     center_angle=SERVO_CENTER
# )
# pid_y = PID(
#     kp=PID_CFG['y']['kp'], ki=PID_CFG['y']['ki'], kd=PID_CFG['y']['kd'],
#     setpoint=SETPOINT_Y,
#     inverted=False,
#     servo_min=SERVO_CENTER - SERVO_LIMIT,
#     servo_max=SERVO_CENTER + SERVO_LIMIT,
#     center_angle=SERVO_CENTER
# )
# print_pid(pid_x, "XPID")
# print_pid(pid_y, "YPID")

# # ── Ball object ───────────────────────────────────────────────────────────────
# ball = create_ball(SETPOINT_X, SETPOINT_Y)
# print_ball(ball)

# # ── HSV values ────────────────────────────────────────────────────────────────
# lower_hsv = np.array(HSV_CFG['min'])
# upper_hsv = np.array(HSV_CFG['max'])

# # ── Centre-crop ROI ───────────────────────────────────────────────────────────
# # Only this region is processed each frame — matches original's controlROI
# roi_x1 = SETPOINT_X - CONTROL_AREA // 2
# roi_y1 = SETPOINT_Y - CONTROL_AREA // 2
# roi_x2 = SETPOINT_X + CONTROL_AREA // 2
# roi_y2 = SETPOINT_Y + CONTROL_AREA // 2

# # ── Centre servos (handshake) ─────────────────────────────────────────────────
# send_angles(SERVO_CENTER, SERVO_CENTER)
# time.sleep(0.5)

# input("\nAll parameters set. Press Enter to start...\n")

# # ── CSV log ───────────────────────────────────────────────────────────────────
# csv_file   = open('pid_output_data.csv', 'w', newline='')
# csv_writer = csv.writer(csv_file)
# csv_writer.writerow(['kp', pid_x.kp, 'ki', pid_x.ki, 'kd', pid_x.kd])
# csv_writer.writerow(['x_setpoint', SETPOINT_X, 'y_setpoint', SETPOINT_Y])
# csv_writer.writerow(['frame', 'x_ball', 'x_p', 'x_i', 'x_d', 'x_out',
#                               'y_ball', 'y_p', 'y_i', 'y_d', 'y_out'])

# # ─────────────────────────────────────────────────────────────────────────────
# # MAIN LOOP
# # ─────────────────────────────────────────────────────────────────────────────
# print(f"Running in {'DEBUG' if MODE == 'debug' else 'STANDARD'} mode. Press Q to quit.\n")

# frame_count = 0
# t_start     = time.time()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         continue

#     # crop to centre ROI — only process this region
#     roi  = frame[roi_y1:roi_y2, roi_x1:roi_x2]

#     # HSV threshold
#     hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
#     mask = morph_ops(mask)

#     # ball detection + physics update
#     track_filtered_object(ball, mask, FRAME_W, FRAME_H, CONTROL_AREA)

#     if ball.detected:
#         # PID compute (both axes)
#         pid_compute(pid_x, pid_y, ball)

#         # send angles to Arduino
#         send_angles(pid_x.output[0], pid_y.output[0])

#         # log to CSV
#         csv_writer.writerow([
#             frame_count,
#             ball.x[0], pid_x.P, pid_x.I, pid_x.D, pid_x.output[0],
#             ball.y[0], pid_y.P, pid_y.I, pid_y.D, pid_y.output[0]
#         ])

#     # display
#     if MODE == 'debug':
#         draw_overlay(frame, ball, SETPOINT_X, SETPOINT_Y,
#                      CONTROL_AREA, pid_x, pid_y)
#         cv2.imshow('Mask', mask)

#     cv2.imshow('Ball Balancing PID System', frame)
#     frame_count += 1

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # ─────────────────────────────────────────────────────────────────────────────
# # EXIT
# # ─────────────────────────────────────────────────────────────────────────────
# t_end   = time.time()
# elapsed = t_end - t_start
# print(f"\n{frame_count} frames | {elapsed:.1f}s | {frame_count/elapsed:.1f} FPS avg")

# csv_file.close()
# cap.release()
# cv2.destroyAllWindows()

# # return servos to centre on exit
# send_angles(SERVO_CENTER, SERVO_CENTER)
# time.sleep(0.5)
# arduino.close()
# print("Closed. Goodbye.")


# SERVO Y AT 120 degrees

"""
main.py
Entry point — three modes selectable via command line argument.

Usage:
  python main.py            # standard mode (no overlay, clean loop)
  python main.py debug      # standard + overlay + mask window
  python main.py hsv        # HSV tuner only (saves values to config.json)

Architecture:
  Laptop  : camera → ball detection → physics filter → PID → serial
  Arduino : receives two angles over serial → drives servoX and servoY

Serial protocol:
  Python sends:  "90,85\n"   (X angle, Y angle as comma-separated integers)
  Arduino reads: parses comma, writes each to servo

Coordinate system:
  (0,0) = top-left of frame
  setpoint = frame centre (320, 240) at 640x480
  error > 0 means ball is below/right of setpoint
"""

import cv2
import numpy as np
import serial
import time
import json
import sys
import csv

from ball_physic  import create_ball, print_ball
from ball_tracker import track_filtered_object, morph_ops, draw_overlay
from pid          import PID, pid_compute, change_setpoint, print_pid, saturation_filter


# ── Load config ───────────────────────────────────────────────────────────────
with open('config.json') as f:
    cfg = json.load(f)

PID_CFG  = cfg['pid']
HSV_CFG  = cfg['hsv']
SRV_CFG  = cfg['servo']
CAM_IDX  = cfg['camera_index']
PORT     = cfg['serial_port']
BAUD     = cfg['baud_rate']
FRAME_W      = 640
FRAME_H      = 480
CONTROL_AREA = 400          # px — only the centre 400x400 region is processed
SETPOINT_X   = (FRAME_W // 2)-15
SETPOINT_Y   = (FRAME_H // 2)-15
SERVO_CENTER_X = SRV_CFG['center_x']    # 90
SERVO_CENTER_Y = SRV_CFG['center_y']    # 120
SERVO_LIMIT  = SRV_CFG['limit']     # 20 → range is 70–110

MODE = sys.argv[1] if len(sys.argv) > 1 else 'standard'


# ── Serial ────────────────────────────────────────────────────────────────────
print(f"\nOpening serial on {PORT} at {BAUD} baud...")
arduino = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)                       # wait for Arduino reset
arduino.reset_input_buffer()
print("Serial ready.\n")


def send_angles(ax, ay):
    """Clamp and send two servo angles to Arduino over serial."""
    ax = int(saturation_filter(ax, SERVO_CENTER_X - SERVO_LIMIT, SERVO_CENTER_X + SERVO_LIMIT))
    ay = int(saturation_filter(ay, SERVO_CENTER_Y - SERVO_LIMIT, SERVO_CENTER_Y + SERVO_LIMIT))
    arduino.write(f"{ax},{ay}\n".encode())
    arduino.flush()


# ── Camera ────────────────────────────────────────────────────────────────────
print(f"Opening camera index {CAM_IDX}...")
cap = cv2.VideoCapture(CAM_IDX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)       # reduce latency
if not cap.isOpened():
    print("ERROR: Could not open camera. Check camera_index in config.json.")
    arduino.close()
    exit(1)
print("Camera ready.\n")


# ─────────────────────────────────────────────────────────────────────────────
# HSV TUNER MODE
# ─────────────────────────────────────────────────────────────────────────────
if MODE == 'hsv':
    print("HSV Tuner mode. Adjust sliders until only the ball is white in the mask.")
    print("Press S to save and exit. Press Q to exit without saving.\n")

    def _nothing(x): pass

    cv2.namedWindow('HSV Tuner')
    cv2.createTrackbar('H min', 'HSV Tuner', HSV_CFG['min'][0], 179, _nothing)
    cv2.createTrackbar('H max', 'HSV Tuner', HSV_CFG['max'][0], 179, _nothing)
    cv2.createTrackbar('S min', 'HSV Tuner', HSV_CFG['min'][1], 255, _nothing)
    cv2.createTrackbar('S max', 'HSV Tuner', HSV_CFG['max'][1], 255, _nothing)
    cv2.createTrackbar('V min', 'HSV Tuner', HSV_CFG['min'][2], 255, _nothing)
    cv2.createTrackbar('V max', 'HSV Tuner', HSV_CFG['max'][2], 255, _nothing)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo   = np.array([cv2.getTrackbarPos('H min', 'HSV Tuner'),
                         cv2.getTrackbarPos('S min', 'HSV Tuner'),
                         cv2.getTrackbarPos('V min', 'HSV Tuner')])
        hi   = np.array([cv2.getTrackbarPos('H max', 'HSV Tuner'),
                         cv2.getTrackbarPos('S max', 'HSV Tuner'),
                         cv2.getTrackbarPos('V max', 'HSV Tuner')])
        mask = cv2.inRange(hsv, lo, hi)
        cv2.imshow('HSV Tuner', frame)
        cv2.imshow('Mask',      mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            cfg['hsv']['min'] = lo.tolist()
            cfg['hsv']['max'] = hi.tolist()
            with open('config.json', 'w') as f:
                json.dump(cfg, f, indent=2)
            print(f"Saved HSV min={lo.tolist()} max={hi.tolist()} to config.json")
            break
        elif key == ord('q'):
            print("Exiting without saving.")
            break

    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
    exit(0)


# ─────────────────────────────────────────────────────────────────────────────
# STANDARD / DEBUG MODE
# ─────────────────────────────────────────────────────────────────────────────

# ── PID objects ───────────────────────────────────────────────────────────────
print("Creating PID controllers...")
pid_x = PID(
    kp=PID_CFG['x']['kp'], ki=PID_CFG['x']['ki'], kd=PID_CFG['x']['kd'],
    setpoint=SETPOINT_X,
    inverted=False,                              # flip if ball moves wrong way on X
    servo_min=SERVO_CENTER_X - SERVO_LIMIT,
    servo_max=SERVO_CENTER_X + SERVO_LIMIT,
    center_angle=SERVO_CENTER_X
)
pid_y = PID(
    kp=PID_CFG['y']['kp'], ki=PID_CFG['y']['ki'], kd=PID_CFG['y']['kd'],
    setpoint=SETPOINT_Y,
    inverted=True,
    servo_min=SERVO_CENTER_Y - SERVO_LIMIT,
    servo_max=SERVO_CENTER_Y + SERVO_LIMIT,
    center_angle=SERVO_CENTER_Y 
)
print_pid(pid_x, "XPID")
print_pid(pid_y, "YPID")

# ── Ball object ───────────────────────────────────────────────────────────────
ball = create_ball(SETPOINT_X, SETPOINT_Y)
print_ball(ball)

# ── HSV values ────────────────────────────────────────────────────────────────
lower_hsv = np.array(HSV_CFG['min'])
upper_hsv = np.array(HSV_CFG['max'])

# ── Centre-crop ROI ───────────────────────────────────────────────────────────
# Only this region is processed each frame — matches original's controlROI
roi_x1 = SETPOINT_X - CONTROL_AREA // 2
roi_y1 = SETPOINT_Y - CONTROL_AREA // 2
roi_x2 = SETPOINT_X + CONTROL_AREA // 2
roi_y2 = SETPOINT_Y + CONTROL_AREA // 2

# ── Centre servos (handshake) ─────────────────────────────────────────────────
send_angles(SERVO_CENTER_X, SERVO_CENTER_Y)
time.sleep(0.5)

input("\nAll parameters set. Press Enter to start...\n")

# ── CSV log ───────────────────────────────────────────────────────────────────
csv_file   = open('pid_output_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['kp', pid_x.kp, 'ki', pid_x.ki, 'kd', pid_x.kd])
csv_writer.writerow(['x_setpoint', SETPOINT_X, 'y_setpoint', SETPOINT_Y])
csv_writer.writerow(['frame', 'x_ball', 'x_p', 'x_i', 'x_d', 'x_out',
                              'y_ball', 'y_p', 'y_i', 'y_d', 'y_out'])

# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
print(f"Running in {'DEBUG' if MODE == 'debug' else 'STANDARD'} mode. Press Q to quit.\n")

frame_count = 0
t_start     = time.time()


# ── PID monitor / tuning metrics ─────────────────────────────────────────────
# Display/logging only. These values do not change the existing PID control.
last_loop_time = 0.0
current_fps = 0.0

settling_threshold = 10       # pixels from setpoint
settling_hold_time = 0.50     # seconds inside the threshold
settling_start = None
settling_time = None

max_error_x = 0
max_error_y = 0

previous_error_x = None
previous_error_y = None
oscillation_x = 0
oscillation_y = 0

steady_error_x = 0
steady_error_y = 0

metric_start_time = time.perf_counter()


def draw_pid_interface(frame, ball, pid_x, pid_y):
    """Display the tuning interface using the existing PID/detection values."""
    global last_loop_time, current_fps
    global max_error_x, max_error_y
    global previous_error_x, previous_error_y
    global oscillation_x, oscillation_y
    global settling_start, settling_time
    global steady_error_x, steady_error_y

    if ball.detected:
        # Same error convention used by the project:
        # setpoint - measured ball position.
        error_x = SETPOINT_X - ball.x[0]
        error_y = SETPOINT_Y - ball.y[0]

        abs_x = abs(error_x)
        abs_y = abs(error_y)

        max_error_x = max(max_error_x, abs_x)
        max_error_y = max(max_error_y, abs_y)

        steady_error_x = abs_x
        steady_error_y = abs_y

        # Simple oscillation indicator: count sign changes in error.
        if previous_error_x is not None:
            if (previous_error_x > 0 > error_x) or (previous_error_x < 0 < error_x):
                oscillation_x += 1
            if (previous_error_y > 0 > error_y) or (previous_error_y < 0 < error_y):
                oscillation_y += 1

        previous_error_x = error_x
        previous_error_y = error_y

        # Settling time: enter ±10 px in both axes and remain there for 0.5 s.
        now = time.perf_counter()
        inside_band = abs_x <= settling_threshold and abs_y <= settling_threshold

        if inside_band:
            if settling_start is None:
                settling_start = now
            elif settling_time is None and (now - settling_start) >= settling_hold_time:
                settling_time = settling_start - metric_start_time
        else:
            settling_start = None
            settling_time = None

        # Target, ball and error vector.
        cv2.drawMarker(
            frame, (SETPOINT_X, SETPOINT_Y),
            (0, 0, 255), cv2.MARKER_CROSS, 30, 2
        )
        cv2.circle(
            frame, (int(ball.x[0]), int(ball.y[0])),
            10, (0, 255, 0), 2
        )
        cv2.line(
            frame,
            (int(ball.x[0]), int(ball.y[0])),
            (SETPOINT_X, SETPOINT_Y),
            (255, 0, 0), 2
        )

    else:
        error_x = 0
        error_y = 0
        steady_error_x = 0
        steady_error_y = 0

    # Display panel.
    overlay = frame.copy()
    cv2.rectangle(overlay, (5, 5), (310, 365), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    if ball.detected:
        ball_x_display = int(ball.x[0])
        ball_y_display = int(ball.y[0])
        servo_x_display = int(pid_x.output[0])
        servo_y_display = int(pid_y.output[0])
    else:
        ball_x_display = "---"
        ball_y_display = "---"
        servo_x_display = SERVO_CENTER_X
        servo_y_display = SERVO_CENTER_Y

    settling_display = f"{settling_time:.2f}s" if settling_time is not None else "--"

    lines = [
        f"Target: ({SETPOINT_X}, {SETPOINT_Y})",
        f"Ball:   ({ball_x_display}, {ball_y_display})",
        f"Error X: {error_x:+d} px",
        f"Error Y: {error_y:+d} px",
        "",
        f"Kp X: {pid_x.kp:.4f}",
        f"Ki X: {pid_x.ki:.4f}",
        f"Kd X: {pid_x.kd:.4f}",
        f"Kp Y: {pid_y.kp:.4f}",
        f"Ki Y: {pid_y.ki:.4f}",
        f"Kd Y: {pid_y.kd:.4f}",
        "",
        f"Servo X: {servo_x_display} deg",
        f"Servo Y: {servo_y_display} deg",
        f"Loop: {last_loop_time * 1000:.1f} ms",
        f"FPS: {current_fps:.1f}",
        "",
        f"Max error X: {max_error_x:.0f} px",
        f"Max error Y: {max_error_y:.0f} px",
        f"Steady error: ({steady_error_x:.0f}, {steady_error_y:.0f})",
        f"Settling: {settling_display}",
        f"Oscillations X/Y: {oscillation_x}/{oscillation_y}",
    ]

    y = 25
    for line in lines:
        cv2.putText(
            frame, line, (15, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48,
            (255, 255, 255), 1, cv2.LINE_AA
        )
        y += 15

while True:
    loop_start = time.perf_counter()
    ret, frame = cap.read()
    if not ret:
        continue

    print("frame shape:", frame.shape, "roi coords:", roi_x1, roi_y1, roi_x2, roi_y2)

    roi  = frame[roi_y1:roi_y2, roi_x1:roi_x2]

    print("roi shape:", roi.shape)
    # crop to centre ROI — only process this region

    # HSV threshold
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = morph_ops(mask)

    # ball detection + physics update
    track_filtered_object(ball, mask, FRAME_W, FRAME_H, CONTROL_AREA)

    if ball.detected:
        # PID compute (both axes)
        pid_compute(pid_x, pid_y, ball)

        # send angles to Arduino
        send_angles(pid_x.output[0], pid_y.output[0])

        # log to CSV
        csv_writer.writerow([
            frame_count,
            ball.x[0], pid_x.P, pid_x.I, pid_x.D, pid_x.output[0],
            ball.y[0], pid_y.P, pid_y.I, pid_y.D, pid_y.output[0]
        ])

    # display
    if MODE == 'debug':
        draw_overlay(frame, ball, SETPOINT_X, SETPOINT_Y,
                     CONTROL_AREA, pid_x, pid_y)
        cv2.imshow('Mask', mask)

    # ── PID tuning interface ────────────────────────────────────────────────
    # Display-only: existing detection, PID calculation and serial code above
    # are unchanged.
    now = time.perf_counter()
    last_loop_time = now - loop_start
    current_fps = 1.0 / last_loop_time if last_loop_time > 0 else 0.0

    draw_pid_interface(frame, ball, pid_x, pid_y)

    cv2.imshow('Ball Balancing PID System', frame)
    frame_count += 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ─────────────────────────────────────────────────────────────────────────────
# EXIT
# ─────────────────────────────────────────────────────────────────────────────
t_end   = time.time()
elapsed = t_end - t_start
print(f"\n{frame_count} frames | {elapsed:.1f}s | {frame_count/elapsed:.1f} FPS avg")

csv_file.close()
cap.release()
cv2.destroyAllWindows()

# return servos to centre on exit
send_angles(SERVO_CENTER_X, SERVO_CENTER_Y)
time.sleep(0.5)
arduino.close()
print("Closed. Goodbye.")
