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
    '''Recursive model of the form Y = A@X + B with noise covariance Q.

    args:

        get_A(dt): Model scalar matrix
            2D numpy array generator that takes dt as input
        get_B(dt): Model shift
            2D numpy array generator that takes dt as input
        get_Q(dt): Model noise covariance matrix.
            2D numpy array generator that takes dt as input
    '''
    def __init__(self,
                 A_init: NDArray[np.float64],
                 B_init: NDArray[np.float64],
                 Q_init: NDArray[np.float64]):
        self.A = A_init
        self.B = B_init
        self.Q = Q_init


class LinearModel_RollPitch(LinearModel):
    def get_tangent_scale(self, dt:float):
        '''
        Shifts the model prediction given table state and time passed
        args:
            dt: Time (ms) passed since the last estimate
        '''
        return np.array([[1,0,dt,0],
                        [0,1,0,dt],
                        [0,0,1,0],
                        [0,0,0,1]])

    def get_tangent_shift(self, dt:float, roll, pitch):
        '''
        Shifts the model prediction given table state and time passed
        args:
            dt: Time (ms) passed since the last estimate
            roll: Table roll in degrees (cw rotation seen from x+)
            pitch: Table pitch in degrees (cw rotation seen from y+)
        '''
        g = 9.8
        c = 1 #TODO: find sphere radius constant
        return g * c * np.array([-np.sin(2*pitch) * dt * dt,
                                np.sin(2*roll) * dt * dt,
                                -2 * np.sin(2*pitch) * dt,
                                2 * np.sin(2*roll) * dt])

    def get_guass_Q(self,dt:float):
        '''
        This Q model noise covariance assumes equal, independent, gaussian
        acceleration noise on each axis (x,y)
        args:
            dt: (float) Time (ms) passed since the last estimate
        '''
        a = dt * dt * dt * dt / 4
        b = dt * dt * dt / 2
        c = dt * dt
        acc_var = 0 # TODO: the single axis acceleration variance of the model (for Q)
        return acc_var * np.array([[a,0,b,0],
                                [0,a,0,b],
                                [b,0,c,0],
                                [0,b,0,c]])
    
    def update_model(self,dt: float,roll: float,pitch: float)->None:
        '''
        Updates the linear models coefficients given environment factors
        args:
            dt: Time (ms) passed since the last estimate
            roll: Table roll in degrees (cw rotation seen from x+)
            pitch: Table pitch in degrees (cw rotation seen from y+)
        '''
        self.A = self.get_tangent_scale(dt)
        self.B = self.get_tangent_shift(dt,roll,pitch)
        self.Q = self.get_guass_Q(dt)


class KalmanFilter:
    def __init__(self,model,P_init:np.ndarray):
        self.model = model
        self.P_prev = self.P = self.P_minus = P_init # initial estimate noise

        self.H = np.array([[1,0,0,0],
                           [0,1,0,0]])

        self.state_est_prev = np.zeros((4,1)) # x,y,vx,vy
        self.state_est_minus = np.ndarray((4,1)) # x,y,vx,vy
        self.state_est = np.ndarray((4,1)) # x,y,vx,vy

    def h_predict(self,dt,roll,pitch):
        '''
        Estimate the current state using the model and table params
        '''
        self.model.set_state(dt,roll,pitch)
        self.state_est_minus = self.model.A @ self.state_est_prev + self.model.B
        self.P_minus = self.model.A @ self.model.P @ self.model.A.T + self.model.Q
        return self.state_est_minus

    def h_update(self,meas: NDArray[np.float64], meas_cov):
        '''
        Update the estimate with a measurement
        '''
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + meas_cov)
        self.state_est = self.state_est_minus + K @ (meas - self.state_est_minus)
        self.state_est_prev = self.state_est
        self.P = (np.identity(self.P.shape[0]) - K @ self.H) @ self.P_minus
        return self.state_est

    def estimate_state(self,dt,roll,pitch,meas,meas_cov):
        self.h_predict(dt,roll,pitch)
        return self.h_update(meas, meas_cov)

# get_A()
def get_tangent_step(dt:float):
    return np.array([[1,0,dt,0],
                     [0,1,0,dt],
                     [0,0,1,0],
                     [0,0,0,1]])

#  get_B()
def get_tangent_shift(dt:float,roll,pitch):
    g = 9.8
    c = 1 #TODO: find sphere radius constant
    return g * c * np.array([-np.sin(2*pitch) * dt * dt,
                             np.sin(2*roll) * dt * dt,
                             -2 * np.sin(2*pitch) * dt,
                             2 * np.sin(2*roll) * dt])


def get_model_acc_variance()->float:
    # TODO: the single axis variance of the model (for Q)
    pass

def get_tangent_Q_cov(dt:float,roll,pitch):
    a = dt * dt * dt * dt / 4
    b = dt * dt * dt / 2
    c = dt * dt
    acc_var = # TODO: the single axis variance of the model (for Q)
    return acc_var * np.array([[a,0,b,0],
                               [0,a,0,b],
                               [b,0,c,0],
                               [0,b,0,c]])

# Initial matrix construction
def measure_camera_mm_covariance()->NDArray[np.float64]:
    # TODO: measure still camera noise (R)
    pass

# initial estimate covariance (P) guess
# this will autocorrect after a few iterations
P_init = np.array([[0.01,0.0,0.0,0.0],
                  [0.0,0.01,0.0,0.0],
                  [0.0,0.0,0.01,0.0],
                  [0.0,0.0,0.0,0.01]])
