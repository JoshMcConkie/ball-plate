import numpy as np
from collections.abc import Callable
from numpy.typing import NDArray

from ball_plate.estimation.models import LinearModel

class KalmanFilter:
    '''
    Kalman filter class for estimating ball state [x,y,vx,vy]
    '''
    def __init__(self, model: LinearModel, P_init: np.ndarray):
        '''
        args:
            model: linear model for predicting model state
            P_init: 4x4 estimate covariance matrix 
        '''
        self.P_prev = self.P = self.P_minus = P_init
        # Transformation matrix from state space -> measurement space (x,y)

        # x,y,vx,vy
        self.state_est_prev = self.state_est_minus = self.state_est = np.zeros((4,1))

    def predict(self,model)->NDArray[np.float64]:
        '''
        Estimate the current state using the model given the table table state and time.
        Alters several instance attributes.

        args:
            dt: Time (ms) passed since the last estimate (𝚫t)
            tilt_about_x: Table tilt about x in degrees (cw seen from x+) (φ)
            tilt_about_y: Table tilt about y in degrees (cw seen from y+) (θ)
            
        output:
            4x1 ball state model prediction
        '''
        self.state_est_minus = model.A @ self.state_est_prev + model.B
        self.P_minus = model.A @ self.P @ model.A.T + model.Q
        
        return self.state_est_minus

    def revise_prediction(self, meas: NDArray[np.float64],
                 meas_cov: NDArray[np.float64], H: NDArray[np.float64])->NDArray[np.float64]:
        '''
        Revise the instances state prediction with measurement data using Kalman Gain.
        Alters several instance attributes.
        
        args:
            meas: 2x1 ball position measurement in mm, aka Z_n
            meas_cov: 2x2 ball position covariance matrix, aka R
            Transformation matrix from state space -> measurement space (x,y)

        output:
            4x1 weighted model-measurement ball state estimate
        '''
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + meas_cov)
        self.state_est = self.state_est_minus + K @ (meas - self.state_est_minus)
        self.state_est_prev = self.state_est
        self.P = (np.identity(self.P.shape[0]) - K @ H) @ self.P_minus
        return self.state_est

    def estimate_state(self, model, meas, meas_cov, H):
        '''
        Wraps prediction and revision steps.
            dt: Time (ms) passed since the last estimate (𝚫t)
            tilt_about_x: Table tilt about x in degrees (cw seen from x+), (φ)
            tilt_about_y: Table tilt about y in degrees (cw seen from y+), (θ)
            meas: 2x1 ball position measurement in mm, (Z_n)
            meas_cov: 2x2 ball position covariance matrix, (R)
        '''
        self.predict(model)
        return self.revise_prediction(meas, meas_cov, H)
