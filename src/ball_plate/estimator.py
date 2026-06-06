from ball_plate.state import *
from math import atan2, sqrt, pi


def estimate_table_angle(table_old:TableState, imu_new:IMUReading):
    # TODO: Estimate table roll and pitch from both imu acc and gyro data
    roll_deg_acc  = atan2(imu_new.ay, imu_new.az) * 180.0 / pi
    pitch_deg_acc = atan2(-imu_new.ax, sqrt(imu_new.ay*imu_new.ay + imu_new.az*imu_new.az)) * 180.0 / pi
    roll = roll_deg_acc
    pitch = pitch_deg_acc
    return roll, pitch

def get_table_state(table_old:TableState, imu_new:IMUReading)->TableState:
    
    roll, pitch = estimate_table_angle(table_old, imu_new)
    roll_rate = (roll-table_old.roll) / (imu_new.timestamp-table_old.timestamp)
    pitch_rate = (pitch-table_old.pitch) / (imu_new.timestamp-table_old.timestamp)
    return TableState(imu_new.timestamp,
                      roll, pitch,
                      roll_rate, pitch_rate)


def get_ball_state(ball_state_old: BallState,
                   ball_meas_new: BallMeasurement)->BallState:
    # TODO: Estimate ball state given measured values from BallMeasurement
    # TODO: Kalman Filter
    x,y = ball_meas_new.x_m, ball_meas_new.y_m
    vx = (x-ball_state_old.x) / (ball_meas_new.timestamp - ball_state_old.timestamp)
    vy = (y-ball_state_old.y) / (ball_meas_new.timestamp - ball_state_old.timestamp)
    return BallState(ball_meas_new.timestamp,
                     x, y, vx, vy)



