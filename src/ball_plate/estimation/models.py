'''
Models for state estimation
'''

import numpy as np
from numpy.typing import NDArray

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
