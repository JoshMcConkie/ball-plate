from ball_plate.state import *

def get_ball_state(ball_state_old: BallState,
                   ball_meas_new: BallMeasurement)->BallState:
    # TODO: Estimate ball state given measured values from BallMeasurement
    # TODO: Kalman Filter
    x,y = ball_meas_new.x_m, ball_meas_new.y_m
    dt = ball_meas_new.timestamp - ball_state_old.timestamp
    if dt <= 0:
        # No time elapsed (same measurement processed twice); keep previous velocity.
        return BallState(ball_meas_new.timestamp,
                         x, y, ball_state_old.vx, ball_state_old.vy)
    vx = (x-ball_state_old.x) / dt
    vy = (y-ball_state_old.y) / dt
    return BallState(ball_meas_new.timestamp,
                     x, y, vx, vy)
