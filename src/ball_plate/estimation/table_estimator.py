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
    # TODO: Estimate table tilt from imu acc/gyro data and expected angle
    tilt_about_x_deg_acc = atan2(imu_new.ay, imu_new.az) * 180.0 / pi
    tilt_about_y_deg_acc = atan2(-imu_new.ax, sqrt(imu_new.ay*imu_new.ay + imu_new.az*imu_new.az)) * 180.0 / pi
    tilt_about_x = tilt_about_x_deg_acc
    tilt_about_y = tilt_about_y_deg_acc
    return tilt_about_x, tilt_about_y

def get_table_state(table_old:TableState, imu_new:IMUReading)->TableState:
    
    tilt_about_x, tilt_about_y = estimate_table_angle(table_old, imu_new)
    dt = imu_new.timestamp - table_old.timestamp
    if dt <= 0:
        # No time elapsed (duplicate/stale reading); keep previous rates.
        return TableState(imu_new.timestamp,
                          tilt_about_x, tilt_about_y,
                          table_old.tilt_about_x_rate, table_old.tilt_about_y_rate)
    tilt_about_x_rate = (tilt_about_x-table_old.tilt_about_x) / dt
    tilt_about_y_rate = (tilt_about_y-table_old.tilt_about_y) / dt
    return TableState(imu_new.timestamp,
                      tilt_about_x, tilt_about_y,
                      tilt_about_x_rate, tilt_about_y_rate)
