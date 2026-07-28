from ball_plate.state import *
from math import atan2, sqrt, pi
import numpy as np
from collections.abc import Callable
from numpy.typing import NDArray


def estimate_table_angle(table_old:TableState, imu_new:IMUReading):
    # TODO: Estimate table roll and pitch from both imu acc and gyro data
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


#=====Kalman Filter =======

class LinearModel:
    '''Recursive model of the form X_pred = A@X + B with noise covariance Q.

    args:

        A_init: 4x4 scaling (Jacobian) matrix
        B_init: 4x1 shift matrix
        Q_init: 4x4 noise covariance matrix.
    '''
    def __init__(self,
                 A_init: NDArray[np.float64],
                 B_init: NDArray[np.float64],
                 Q_init: NDArray[np.float64]):
        self.A = A_init
        self.B = B_init
        self.Q = Q_init

    def update(self,*args,**kwargs)->None:
        raise(NotImplementedError)


class LinearModel_RollPitch(LinearModel):
    def get_tangent_scale(self, dt: float):
        '''
        Shifts the model prediction given table state and time passed
        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
        '''
        return np.array([[1,0,dt,0],
                        [0,1,0,dt],
                        [0,0,1,0],
                        [0,0,0,1]])

    def get_tangent_shift(self, dt: float, roll: float, pitch: float)->NDArray[np.float64]:
        '''
        Shifts the model prediction given table state and time passed
        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
            roll: Table roll in degrees (cw rotation seen from x+), (φ)
            pitch: Table pitch in degrees (cw rotation seen from y+), (θ)
        '''
        g = 9.8
        c = 1 #TODO: find sphere radius constant
        return g * c * np.array([-np.sin(2*pitch) * dt * dt,
                                np.sin(2*roll) * dt * dt,
                                -2 * np.sin(2*pitch) * dt,
                                2 * np.sin(2*roll) * dt])

    def get_guass_Q(self,dt: float)->NDArray[np.float64]:
        '''
        This Q model noise covariance assumes equal, independent, gaussian
        acceleration noise on each axis (x,y)

        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
        '''
        a = dt * dt * dt * dt / 4
        b = dt * dt * dt / 2
        c = dt * dt
        acc_var = 0 # TODO: the single axis acceleration variance of the model (for Q)
        return acc_var * np.array([[a,0,b,0],
                                [0,a,0,b],
                                [b,0,c,0],
                                [0,b,0,c]])
    
    def update(self, dt: float,roll: float,pitch: float)->None:
        '''
        Updates the linear models coefficients given environment factors

        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
            roll: Table roll in degrees (cw rotation seen from x+), (φ)
            pitch: Table pitch in degrees (cw rotation seen from y+), (θ)
        '''
        self.A = self.get_tangent_scale(dt)
        self.B = self.get_tangent_shift(dt,roll,pitch)
        self.Q = self.get_guass_Q(dt)


class KalmanFilter:
    def __init__(self, model: LinearModel, P_init: np.ndarray):
        self.model = model
        self.P_prev = self.P = self.P_minus = P_init # initial estimate noise

        self.H = np.array([[1,0,0,0],
                           [0,1,0,0]])

        # x,y,vx,vy
        self.state_est_prev = self.state_est_minus = np.zeros((4,1))
        self.state_est = np.zeros((4,1))

    def predict(self, dt: float, roll: float, pitch:float)->NDArray[np.float64]:
        '''
        Estimate the current state using the model given the table table state and time.

        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
            roll: Table roll in degrees (cw rotation seen from x+) (φ)
            pitch: Table pitch in degrees (cw rotation seen from y+) (θ)
        '''
        self.model.update(dt,roll,pitch) # update A,B,Q in linear model
        self.state_est_minus = self.model.A @ self.state_est_prev + self.model.B
        self.P_minus = self.model.A @ self.P @ self.model.A.T + self.model.Q
        return self.state_est_minus

    def revise_prediction(self, meas: NDArray[np.float64],
                 meas_cov: NDArray[np.float64])->NDArray[np.float64]:
        '''
        Revise the instances state prediction with measurement data using Kalman Gain.
        
        args:
            meas: 2x1 ball position measurement in mm, aka Z_n
            meas_cov: 2x2 ball position covariance matrix, aka R

        output:
            4x1 composite ball state estimate
        '''
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + meas_cov)
        self.state_est = self.state_est_minus + K @ (meas - self.state_est_minus)
        self.state_est_prev = self.state_est
        self.P = (np.identity(self.P.shape[0]) - K @ self.H) @ self.P_minus
        return self.state_est

    def estimate_state(self,dt,roll,pitch,meas,meas_cov):
        '''
        Wraps prediction and revision steps.
            dt: Time (ms) passed since the last estimate (𝚫t)
            roll: Table roll in degrees (cw rotation seen from x+), (φ)
            pitch: Table pitch in degrees (cw rotation seen from y+), (θ)
            meas: 2x1 ball position measurement in mm, (Z_n)
            meas_cov: 2x2 ball position covariance matrix, (R)
        '''
        self.predict(dt,roll,pitch)
        return self.revise_prediction(meas, meas_cov)

# ======= Initial matrix construction
def measure_camera_mm_covariance()->NDArray[np.float64]:
    # TODO: measure still camera noise (R)
    pass

# initial estimate covariance (P) guess
# this will autocorrect after a few iterations
P_init = np.array([[0.01,0.0,0.0,0.0],
                  [0.0,0.01,0.0,0.0],
                  [0.0,0.0,0.01,0.0],
                  [0.0,0.0,0.0,0.01]])
