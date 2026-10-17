# Ported and adapted from giusenso/Ball-Balancing-PID-System (MIT License)
# Original: https://github.com/giusenso/Ball-Balancing-PID-System

"""
ball_physic.py
Port of ball_physic.c — ball position/velocity tracking with moving average filter

Ball_t struct equivalent:
  detected    : bool
  x[8], y[8] : position history ring buffer (index 0 = newest)
  dx[8],dy[8]: velocity history ring buffer
  smooth_dx   : low-pass filtered X velocity (fed to PID D term)
  smooth_dy   : low-pass filtered Y velocity (fed to PID D term)
"""

N = 3  # moving average window size


class Ball:
    def __init__(self, x, y):
        self.detected   = False
        self.x          = [x] * 8
        self.y          = [y] * 8
        self.dx         = [0] * 8
        self.dy         = [0] * 8
        self.smooth_dx  = 0
        self.smooth_dy  = 0


def create_ball(x, y):
    return Ball(x, y)


def update_ball(ball, new_x, new_y):
    """
    Shift position/velocity history back one slot, insert new values at index 0.
    Compute recursive moving average for smooth velocity.
    """
    # shift X position history
    ball.x[7:0:-1] = ball.x[6::-1]
    ball.x[0] = new_x

    # instantaneous X velocity
    ball.dx[7:0:-1] = ball.dx[6::-1]
    ball.dx[0] = ball.x[0] - ball.x[1]

    # recursive moving average — low-pass filter on velocity
    ball.smooth_dx = (N * ball.smooth_dx - ball.dx[N] + ball.dx[0]) / N

    # shift Y position history
    ball.y[7:0:-1] = ball.y[6::-1]
    ball.y[0] = new_y

    # instantaneous Y velocity
    ball.dy[7:0:-1] = ball.dy[6::-1]
    ball.dy[0] = ball.y[0] - ball.y[1]

    # recursive moving average
    ball.smooth_dy = (N * ball.smooth_dy - ball.dy[N] + ball.dy[0]) / N


def print_ball(ball):
    print(f"Ball: detected={ball.detected} "
          f"x={ball.x[0]} y={ball.y[0]} "
          f"smooth_dx={ball.smooth_dx:.1f} smooth_dy={ball.smooth_dy:.1f}")
