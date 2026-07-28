import time

import numpy as np

from ball_plate.estimation.kalman_filter import KalmanFilter
from ball_plate.estimation.models import BallOnPlateModel
from ball_plate.state import BallMeasurement, BallState, TableState

class BallEstimator:
    def __init__(self, model: BallOnPlateModel):
        self.model = model
        estimate_cov = .01 * np.identity(4) # TODO: find good initial estimate covariance (P)
        self.filter = KalmanFilter(estimate_cov)
        self.meas_cov = 0.01 * np.identity(2) # TODO: measure actual camera tracking covariance
        self.last_timestamp = time.monotonic()

    def estimate_kalman(self, meas: BallMeasurement, table_state: TableState)->BallState:
        '''
        Estimation using a kalman filter
        '''
        dt = time.monotonic() - self.last_timestamp
        self.model.update(dt, table_state.tilt_about_x, table_state.tilt_about_y) # update A,B,Q in linear model
        self.last_timestamp = time.monotonic()
        state = self.filter.estimate_state(self.model, meas=meas.get_meas_vector(), 
                                        meas_cov=self.meas_cov, H=meas.H)
        
        return BallState(time.monotonic(), *state)
    
    def estimate_vanilla(self, ball_state_old: BallState, ball_meas_new: BallMeasurement)->BallState:
        '''
        The original estimate, taking measurement at face value and estimate velocity by differences.
        '''
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
