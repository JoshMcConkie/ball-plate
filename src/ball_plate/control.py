from ball_plate.state import *

KP = 1.0
KI = 0.0
KD = 0.2
MAX_TILT_DEG = 10.0
MAX_INTEGRAL = 100.0

_integral_x = 0.0
_integral_y = 0.0
_last_timestamp = None


def get_command(system: SystemState, ref: ReferenceState)->ControlCommand:
    global _integral_x, _integral_y, _last_timestamp

    error_x = ref.x_goal - system.ball.x
    error_y = ref.y_goal - system.ball.y

    if _last_timestamp is None:
        dt = 0.0
    else:
        dt = system.timestamp - _last_timestamp
        if dt < 0.0:
            dt = 0.0
    _last_timestamp = system.timestamp

    _integral_x += error_x * dt
    _integral_y += error_y * dt
    if _integral_x > MAX_INTEGRAL:
        _integral_x = MAX_INTEGRAL
    elif _integral_x < -MAX_INTEGRAL:
        _integral_x = -MAX_INTEGRAL
    if _integral_y > MAX_INTEGRAL:
        _integral_y = MAX_INTEGRAL
    elif _integral_y < -MAX_INTEGRAL:
        _integral_y = -MAX_INTEGRAL

    # TODO: fix the vx and vy to v_error_x and v_error_y
    pitch_deg = KP * error_x + KI * _integral_x - KD * system.ball.vx
    roll_deg = KP * error_y + KI * _integral_y - KD * system.ball.vy

    if pitch_deg > MAX_TILT_DEG:
        pitch_deg = MAX_TILT_DEG
    elif pitch_deg < -MAX_TILT_DEG:
        pitch_deg = -MAX_TILT_DEG
    if roll_deg > MAX_TILT_DEG:
        roll_deg = MAX_TILT_DEG
    elif roll_deg < -MAX_TILT_DEG:
        roll_deg = -MAX_TILT_DEG

    return ControlCommand(system.timestamp, roll_deg, pitch_deg)

def get_system_state(ball_state: BallState, table_state: TableState, ref_state: ReferenceState)->SystemState:
    timestamp = max(ball_state.timestamp,table_state.timestamp)
    return SystemState(timestamp, ball_state,table_state,ref_state)
