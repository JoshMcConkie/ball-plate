'''
Models for state estimation
'''

from typing import Any

from cv2 import VideoCapture
import numpy as np
from numpy.typing import NDArray

from ball_plate.estimation import calibration

class LinearModel:
    '''Recursive model of the form X_pred = A@X + B with noise covariance Q.

    args:

        A_init: 4x4 scaling (Jacobian) matrix
        B_init: 4x1 shift matrix
        Q_init: 4x4 noise covariance matrix.
    '''
    def __init__(self,
                 A_init: NDArray[np.float64]=np.identity(4),
                 B_init: NDArray[np.float64]=np.zeros((4,1)),
                 Q_init: NDArray[np.float64]=np.identity(4)):
        self.A = A_init
        self.B = B_init
        self.Q = Q_init

    def update(self,*args,**kwargs)->None:
        raise(NotImplementedError)


class BallOnPlateModel(LinearModel):
    def __init__(self, acc_var: float):
        super().__init__()
        self.acc_var = acc_var

    def get_tangent_scale(self, dt: float):
        '''
        Shifts the model prediction given plate state and time passed
        args:
            dt: Time (s) passed since the last estimate (𝚫t)
        '''
        return np.array([[1,0,dt,0],
                        [0,1,0,dt],
                        [0,0,1,0],
                        [0,0,0,1]])

    def get_tangent_shift(self, dt: float, tilt_about_x: float, tilt_about_y: float)->NDArray[np.float64]:
        '''
        Shifts the model prediction given plate state and time passed
        args:
            dt: Time (s) passed since the last estimate (𝚫t)
            tilt_about_x: Plate tilt about x in radians (cw seen from x+), (φ)
            tilt_about_y: Plate tilt about y in radians (cw seen from y+), (θ)
        '''
        g = 9.8 # gravity
        c = 5/7 # Rolling coef
        return g * c * 0.25 * np.array([-np.sin(2*tilt_about_y) * dt * dt,
                                        np.sin(2*tilt_about_x) * dt * dt,
                                        -2 * np.sin(2*tilt_about_y) * dt,
                                        2 * np.sin(2*tilt_about_x) * dt])

    def get_guass_Q(self,dt: float)->NDArray[np.float64]:
        '''
        This Q model noise covariance assumes equal, independent, gaussian
        acceleration noise on each axis (x,y)

        args:
            dt: Time (s) passed since the last estimate (𝚫t)
        '''
        a = dt * dt * dt * dt / 4
        b = dt * dt * dt / 2
        c = dt * dt
        return self.acc_var * np.array([[a,0,b,0],
                                [0,a,0,b],
                                [b,0,c,0],
                                [0,b,0,c]])
    
    def update(self, dt: float, tilt_about_x: float, tilt_about_y: float)->None:
        '''
        Updates the linear models coefficients given environment factors

        args:
            dt: Time (s) passed since the last estimate (𝚫t)
            tilt_about_x: Plate global frame tilt about x in radians (cw seen from x+), (φ)
            tilt_about_y: Plate global frame tilt about y in radians (cw seen from y+), (θ)
        '''
        self.A = self.get_tangent_scale(dt)
        self.B = self.get_tangent_shift(dt, tilt_about_x, tilt_about_y)
        self.Q = self.get_guass_Q(dt)


""" Old IMU work
class PlateModel(LinearModel):
    def __init__(self):
        '''
        args:
            feed: cv2 video stream
            calibrate: false triggers use of previous calibration data (default true)
        '''
        super().__init__()
        # To callibrate the Q matrix calculations, we need to seperate
        # bias noise from gyro white noise using sampling.


    def get_tangent_scale(self, dt: float):
        '''
        Shifts the model prediction given time passed
        args:
            dt: Time (s) passed since the last estimate (𝚫t)
        '''
        return np.array([[1,0,dt,0],
                        [0,1,0,dt],
                        [0,0,1,0],
                        [0,0,0,1]])

    def get_gyro_shift(self, gx:float, gy:float, dt:float):
        pass

    # TODO: Investigate increasing Q when rotational velocity increases
    def get_guass_Q(self, gx:float, gy:float, dt:float)->NDArray[np.float64]:
            '''
            This Q model noise covariance assumes equal, independent, gaussian
            angular acceleration noise on each local plate axis (roll,pitch)

            args:
                gx: measured roll acceleration (rad/s)
                gy: measured pitch acceleration (rad/s)
                dt: Time (s) passed since the last estimate (𝚫t)

            See Estimation Structure notes for explanations of matrix calculations
            '''
            # Variance from A
            
            # Variance matrix from B
            
            raise NotImplementedError
    
    def update(self,ax,ay,az,gx,gy,gz,dt):
        self.A = self.get_tangent_scale(dt)
        self.B = self.get_gyro_shift(gx,gy,dt)
        self.Q = self.get_guass_Q(gx, gy, dt)
"""