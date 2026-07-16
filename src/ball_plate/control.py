from math import sin, asin, radians, degrees
from ball_plate.config import SERVO_ARM_LENGTH, SERVO_CENTER_DEG, TABLE_H_M, TABLE_W_M
from ball_plate.state import *

# Errors are in meters (table is ~0.22 m wide, so |error| <= ~0.11 m).
# Gains must be large enough to turn cm-scale errors into degrees of tilt.
KP = 100.0
KI = 4.0
KD = 40.0
MAX_TILT_DEG = 10.0
MAX_INTEGRAL = MAX_TILT_DEG/KI

_error_x_1 = None
_error_y_1 = None
_error_vx_1 = 0.0
_error_vy_1 = 0.0
alpha = .8

_integral_x = 0.0
_integral_y = 0.0
_last_timestamp = None

def get_servo_angles(roll_deg: float, pitch_deg: float)->tuple[float,float]:
    # Edge lift needed for tilt = (table/2)*sin(tilt); servo arm rotates by asin(lift/arm)
    arg_x = (TABLE_W_M/(2*SERVO_ARM_LENGTH)) * sin(radians(pitch_deg))
    arg_y = (TABLE_H_M/(2*SERVO_ARM_LENGTH)) * sin(radians(roll_deg))
    # Clamp to asin domain in case the requested tilt exceeds the arm's reach.
    # asin(...) is the servo deflection from flat; offset by the firmware
    # neutral (90 deg) so a balanced plate commands flat instead of an extreme.
    servox_deg = SERVO_CENTER_DEG + degrees(asin(max(-1.0, min(1.0, arg_x))))
    servoy_deg = SERVO_CENTER_DEG + degrees(asin(max(-1.0, min(1.0, arg_y))))
    return servox_deg, servoy_deg


def get_command(system: SystemState, ref: ReferenceState)->ControlCommand:
    global _error_x_1, _error_y_1
    global _integral_x, _integral_y, _last_timestamp

    error_x = ref.x_goal - system.ball.x
    error_y = ref.y_goal - system.ball.y

    if _last_timestamp is None:
        dt = 0.0
    else:
        dt = system.timestamp - _last_timestamp
        if dt <= 0.0:
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

    # A sensor state can be reused by more than one control iteration. Do not
    # differentiate it again when its timestamp has not advanced, and suppress
    # the derivative kick on the first sample.
    if dt > 0.0 and _error_x_1 is not None and _error_y_1 is not None:
        error_vx = alpha*(error_x - _error_x_1) / dt + (1-alpha) * _error_vx_1
        error_vy = alpha*(error_y - _error_y_1) / dt + (1-alpha) * _error_vy_1
    else:
        error_vx = 0.0
        error_vy = 0.0

    if dt > 0.0 or _error_x_1 is None:
        _error_x_1 = error_x
        _error_y_1 = error_y
    # Convert the position and velocity error into the requested plate tilt.
    pitch_deg = (KP * error_x + KI * _integral_x + KD * error_vx)
    roll_deg = (KP * error_y + KI * _integral_y + KD * error_vy)

    if pitch_deg > MAX_TILT_DEG:
        pitch_deg = MAX_TILT_DEG
    elif pitch_deg < -MAX_TILT_DEG:
        pitch_deg = -MAX_TILT_DEG
    if roll_deg > MAX_TILT_DEG:
        roll_deg = MAX_TILT_DEG
    elif roll_deg < -MAX_TILT_DEG:
        roll_deg = -MAX_TILT_DEG

    servox_deg, servoy_deg = get_servo_angles(roll_deg, pitch_deg)

    return ControlCommand(system.timestamp, roll_deg, pitch_deg, servox_deg, servoy_deg)

def get_system_state(ball_state: BallState, table_state: TableState, ref_state: ReferenceState)->SystemState:
    timestamp = max(ball_state.timestamp,table_state.timestamp)
    return SystemState(timestamp, ball_state,table_state,ref_state)
