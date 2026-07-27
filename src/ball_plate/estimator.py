from ball_plate.state import *
from math import atan2, sqrt, pi
import numpy as np
from collections.abc import Callable


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

        A(dt): Model scalar matrix
            2D numpy array generator that takes dt as input
        B(dt): Model shift
            2D numpy array generator that takes dt as input
        Q(dt): Model noise covariance matrix.
            2D numpy array generator that takes dt as input
    '''
    def __init__(self,
                 get_a: Callable[[float],np.ndarray],
                 get_B: Callable[[float,float,float],np.ndarray],
                 get_Q: Callable[[float,float,float],np.ndarray]):
        self.get_B = get_a
        self.get_B = get_B
        self.get_Q = get_Q
        self.A = None
        self.B = None
        self.Q = None

    def init_matrices(self,A,B,Q):
        self.A = A
        self.B = B
        self.Q = Q

    def set_state(self,dt,roll,pitch):
        self.A = self.A(dt)
        self.B = self.B(dt,roll,pitch)
        self.Q = self.Q(dt,roll,pitch)




class KalmanFilter:
    def __init__(self,model,P_init:np.ndarray):
        self.model = model

        self.P_prev = P_init
        self.P = P_init
        self.P_minus = P_init

        self.H = np.array([[1,0,0,0],
                           [0,1,0,0]])

        self.state_est_prev = np.zeros((4,1)) # x,y,vx,vy
        self.state_est_minus = np.ndarray((4,1)) # x,y,vx,vy
        self.state_est = np.ndarray((4,1)) # x,y,vx,vy

    def h_predict(self,dt,phi,theta):
        '''
        Estimate the current state using the model and table params
        '''
        self.model.set_state(dt,phi,theta)
        self.state_est_minus = self.model.A @ self.state_est_prev + self.model.B
        self.P_minus = self.model.A @ self.model.P @ self.model.A.T + self.model.Q
        return self.state_est_minus

    def h_update(self,meas: np.ndarray[float], meas_cov):
        '''
        Update the estimate with a measurement
        '''
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + meas_cov)
        self.state_est = self.state_est_minus + K @ (meas - self.state_est_minus)
        self.state_est_prev = self.state_est
        self.P = (np.identity(self.P.shape[0]) - K @ self.H) @ self.P_minus
        return self.state_est

    def estimate_state(self,dt,meas,meas_cov):
        self.h_predict(dt)
        return self.h_update(meas, meas_cov)

def get_tangent_step(dt:float):
    return np.array([[1,0,dt,0],
                     [0,1,0,dt],
                     [0,0,1,0],
                     [0,0,0,1]])
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
def measure_camera_mm_covariance()->np.ndarray[float]:
    # TODO: measure still camera noise (R)
    pass

# initial estimate covariance (P) guess
# this will autocorrect after a few iterations
P_init = np.array([[0.01,0.0,0.0,0.0],
                  [0.0,0.01,0.0,0.0],
                  [0.0,0.0,0.01,0.0],
                  [0.0,0.0,0.0,0.01]])
