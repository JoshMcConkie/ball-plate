
import numpy as np
from numpy.typing import NDArray

from ball_plate.estimation.models import LinearModel


class KalmanFilter:
    '''
    Kalman filter class for estimating ball state [x,y,vx,vy]
    '''
    def __init__(self, P_init: np.ndarray):
        '''
        args:
            model: linear model for predicting model state
            P_init: 4x4 estimate covariance matrix 
        '''
        self.P_prev = self.P = self.P_minus = P_init
        self.state_est_prev = self.state_est_minus = self.state_est = np.zeros((4,1))

    def predict(self,model)->NDArray[np.float64]:
        '''
        Estimate the current state using the model given the table table state and time.
        Alters several instance attributes.

        args:
            model: Linear model for state prediction
            
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
            meas: mx1 measurement in mm, aka Z_n
            meas_cov: mxm covariance matrix, aka R
            H: Transformation matrix from state space -> measurement space

        output:
            4x1 weighted model-measurement state estimate
        '''
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + meas_cov)
        self.state_est = self.state_est_minus + K @ (meas - self.state_est_minus)
        self.state_est_prev = self.state_est
        self.P = (np.identity(self.P.shape[0]) - K @ H) @ self.P_minus
        return self.state_est

    def estimate_state(self, model:LinearModel, meas:NDArray[np.float64],
                       meas_cov:NDArray[np.float64], H: NDArray[np.float64]):
        '''
        Wraps prediction and revision steps.

        args:
            model: Linear model for state prediction
            meas: mx1 measurement in mm, aka Z_n
            meas_cov: mxm covariance matrix, aka R
            H: Transformation matrix from state space -> measurement space
        '''
        self.predict(model)
        return self.revise_prediction(meas, meas_cov, H)
