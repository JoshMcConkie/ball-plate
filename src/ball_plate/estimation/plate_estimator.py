import time
from math import atan2, hypot, pi

import imufusion
import numpy as np

from ball_plate.estimation.models import IMUFusionModel
from ball_plate.state import IMUMeasurement, PlateState


class PlateEstimator:
    def __init__(self, model:IMUFusionModel):
        self.model = model
        # estimate_cov = .5 * np.identity(4) # TODO: improve initial estimate covariance
        # self.filter = KalmanFilter(P_init=estimate_cov)
        self.last_timestamp = time.monotonic()

    def estimate_imufusion(self, meas: IMUMeasurement):
        now = time.monotonic()
        dt = now - self.last_timestamp
        self.last_timestamp = now

        self.model.ahrs.set_sample_period(dt)
        self.model.ahrs.update_no_magnetometer(
            np.rad2deg(meas.gyro_vector),
            meas.acc_vector / 9.80665
        )
        q = self.model.ahrs.get_quaternion()
        euler = imufusion.quaternion_to_euler(q)
        state = euler[:2]
        # raise NotImplementedError
        return PlateState(self.last_timestamp,*state)

        
    def estimate_vanilla_acc_only(self, table_old:PlateState, imu_new:IMUMeasurement):
        '''
        Estimate of table state from differences in acceleration only
        '''
        dt = IMUMeasurement.timestamp - table_old.timestamp

        tilt_about_x = atan2(imu_new.ay, hypot(imu_new.az, imu_new.ax)) * 180.0 / pi
        tilt_about_y = atan2(-imu_new.ax, hypot(imu_new.ay,imu_new.az)) * 180.0 / pi
        # tilt_about_x_rate = (tilt_about_x-table_old.tilt_about_x) / dt
        # tilt_about_y_rate = (tilt_about_y-table_old.tilt_about_y) / dt

        return PlateState(imu_new.timestamp,
                              tilt_about_x, tilt_about_y)
