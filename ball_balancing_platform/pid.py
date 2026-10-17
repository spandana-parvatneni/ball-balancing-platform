# Ported and adapted from giusenso/Ball-Balancing-PID-System (MIT License)
# Original: https://github.com/giusenso/Ball-Balancing-PID-System

"""
pid.py
Port of pid.c — PI-D controller with anti-windup and output rate limiter.

Key design decisions (preserved from original):
  - Derivative is on measurement (smooth_velocity), NOT on error.
    This avoids "derivative kick" when setpoint changes suddenly.
  - Integral is clamped to +-150 pixels (anti-windup).
  - Output is rate-limited per frame to prevent jerky servo movement.
  - inverted flag flips error sign for servos mounted in reverse orientation.

Output unit: degrees (0-180), matching Arduino Servo.write().
Original used microseconds (1000-2000us) — gains must be retuned from scratch.
Start: Kp=0.05, Ki=0.0, Kd=0.02. Tune Kp first, then Kd.
"""


def saturation_filter(value, t_min, t_max):
    """Clamp value between t_min and t_max."""
    return max(t_min, min(t_max, value))


class PID:
    def __init__(self, kp, ki, kd, setpoint, inverted,
                 servo_min, servo_max, center_angle):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.setpoint       = setpoint      # target ball position in pixels
        self.inverted       = inverted      # flip error sign if servo mounted backwards
        self.min            = servo_min     # minimum servo angle (degrees)
        self.max            = servo_max     # maximum servo angle (degrees)
        self.center         = center_angle  # neutral angle, typically 90
        self.dt             = 0.033         # ~30fps; update dynamically if needed
        self.error          = [0, 0]        # [current, previous]
        self.output         = [float(center_angle), float(center_angle)]
        self.P              = 0.0
        self.I              = 0.0
        self.D              = 0.0
        self.integral       = 0.0


def _compute_axis(pid, ball_pos, smooth_vel):
    """Single axis PID computation. Called by pid_compute()."""

    # store previous frame values
    pid.error[1]  = pid.error[0]
    pid.output[1] = pid.output[0]

    # compute error — flip sign if servo is physically inverted
    if not pid.inverted:
        pid.error[0] = pid.setpoint - ball_pos
    else:
        pid.error[0] = ball_pos - pid.setpoint

    # integral with anti-windup saturation
    pid.integral += pid.error[0] * pid.dt
    pid.integral  = saturation_filter(pid.integral, -150, 150)

    # derivative: use smooth velocity, saturated
    # flip sign to match error convention
    vel = saturation_filter(smooth_vel, -150, 150)
    if not pid.inverted:
        vel = -vel

    pid.P = pid.kp * pid.error[0]
    pid.I = pid.ki * pid.integral
    pid.D = pid.kd * (vel / pid.dt)

    # raw output centered on servo neutral angle
    raw = pid.center + pid.P + pid.I + pid.D

    # rate limiter: servo can't jump more than 15 degrees per frame
    raw = saturation_filter(raw, pid.output[1] - 15, pid.output[1] + 15)

    # hard output clamp to servo limits
    pid.output[0] = saturation_filter(raw, pid.min, pid.max)


def pid_compute(pid_x, pid_y, ball):
    """Compute PID for both axes. Call once per frame when ball is detected."""
    _compute_axis(pid_x, ball.x[0], ball.smooth_dx)
    _compute_axis(pid_y, ball.y[0], ball.smooth_dy)


def change_setpoint(pid_x, pid_y, ball, new_x, new_y):
    """
    Move target position to (new_x, new_y).
    Fakes a velocity impulse so the PI-D controller doesn't kick on setpoint change.
    """
    ball.smooth_dx  = new_x - pid_x.setpoint
    pid_x.setpoint  = new_x
    ball.smooth_dy  = new_y - pid_y.setpoint
    pid_y.setpoint  = new_y


def print_pid(pid, name="PID"):
    print(f"{name}: Kp={pid.kp} Ki={pid.ki} Kd={pid.kd} | "
          f"error={pid.error[0]:.1f} integral={pid.integral:.2f} "
          f"output={pid.output[0]:.1f}")
