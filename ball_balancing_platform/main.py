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
SETPOINT_X   = FRAME_W // 2
SETPOINT_Y   = FRAME_H // 2
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

while True:
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
