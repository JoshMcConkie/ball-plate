from math import atan2, sqrt, pi
from ball_plate.state import IMUReading, TableState

# TODO: implement new TableEstimator class
class TableEstimator:
    def __init__(self):
        pass

    def estimate_kalman(self, meas: IMUReading):
        pass

    def estimate_vanilla(self):
        pass

def estimate_table_angle(table_old:TableState, imu_new:IMUReading):
    # TODO: Estimate table roll and pitch from imu acc/gyro data and expected angle
    roll_deg_acc  = atan2(imu_new.ay, imu_new.az) * 180.0 / pi
    pitch_deg_acc = atan2(-imu_new.ax, sqrt(imu_new.ay*imu_new.ay + imu_new.az*imu_new.az)) * 180.0 / pi
    roll = roll_deg_acc
    pitch = pitch_deg_acc
    return roll, pitch

def get_table_state(table_old:TableState, imu_new:IMUReading)->TableState:
    
    roll, pitch = estimate_table_angle(table_old, imu_new)
    dt = imu_new.timestamp - table_old.timestamp
    if dt <= 0:
        # No time elapsed (duplicate/stale reading); keep previous rates.
        return TableState(imu_new.timestamp,
                          roll, pitch,
                          table_old.roll_rate, table_old.pitch_rate)
    roll_rate = (roll-table_old.roll) / dt
    pitch_rate = (pitch-table_old.pitch) / dt
    return TableState(imu_new.timestamp,
                      roll, pitch,
                      roll_rate, pitch_rate)