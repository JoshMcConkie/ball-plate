from math import atan2, sqrt, pi, hypot
import time

import numpy as np
from ball_plate.estimation.kalman_filter import KalmanFilter
from ball_plate.estimation.models import LinearModel, TableTiltModel
from ball_plate.state import IMUMeasurement, TableState


class TableEstimator:
    def __init__(self, model:LinearModel):
        self.model = model
        estimate_cov = .5 * np.identity(4) # TODO: improve initial estimate covariance
        self.filter = KalmanFilter(P_init=estimate_cov)
        self.last_timestamp = time.monotonic()


    def estimate_kalman(self, meas: IMUMeasurement, meas_cov):
        dt = time.monotonic() - self.last_timestamp
        self.last_timestamp = time.monotonic()
        self.model.update(dt, meas.gx, meas.gy)
        state = self.filter.estimate_state(model=self.model,meas=, # TODO: alter so state is [roll,pitch,bias_x,bias_y]
                                           meas_cov=meas_cov, H=meas.H)
        
        return TableState(self.last_timestamp, *state)
        
    def estimate_vanilla_acc_only(self, table_old:TableState, imu_new:IMUMeasurement):
        '''
        Estimate of table state from differences in acceleration only
        '''
        dt = IMUMeasurement.timestamp - table_old.timestamp

        tilt_about_x = atan2(imu_new.ay, hypot(imu_new.az, imu_new.ax)) * 180.0 / pi
        tilt_about_y = atan2(-imu_new.ax, hypot(imu_new.ay,imu_new.az)) * 180.0 / pi
        tilt_about_x_rate = (tilt_about_x-table_old.tilt_about_x) / dt
        tilt_about_y_rate = (tilt_about_y-table_old.tilt_about_y) / dt

        return TableState(imu_new.timestamp,
                              tilt_about_x, tilt_about_y,
                              tilt_about_x_rate, tilt_about_y_rate)
