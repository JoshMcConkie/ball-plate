'''
state.py contains the state dataclasses for various measurements. The general flow is:

BallMeasurement → estimator → BallState
IMUReading      → estimator → TableState
SystemState     → controller → ControlCommand

'''
from dataclasses import dataclass
from typing import ClassVar

import numpy as np
from numpy.typing import NDArray

from ball_plate.config import ReferenceConfig


@dataclass
class IMUMeasurement:                                                 
    timestamp: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float

    @property
    def meas_vector(self):
        return np.array([self.ax,self.ay,self.az,self.gx,self.gy,self.gz])

    @property
    def acc_vector(self):
        return np.array([self.ax,self.ay,self.az])

    @property
    def gyro_vector(self):
        return np.array([self.ax,self.ay,self.az])

@dataclass
class BallMeasurement:
    H: ClassVar[NDArray[np.float64]] = np.array([[1.0,0.0,0.0,0.0], # ball state -> measurement state
                                                [0.0,1.0,0.0,0.0]])
    timestamp: float
    x_px: int   # pixel coordinates
    y_px: int   
    x_m: float  # table coordinates
    y_m: float
    radius_px: int
    found: bool
    def get_meas_vector(self):
        return np.array([self.x_m,self.y_m])

@dataclass
class PlateState:
    timestamp: float
    tilt_about_x: float
    tilt_about_y: float
    # tilt_about_x_rate: float
    # tilt_about_y_rate: float

@dataclass
class BallState:
    timestamp: float
    x: float
    y: float
    vx: float
    vy: float

@dataclass
class SystemState:
    timestamp: float
    ball: BallState
    table: PlateState
    ref_state: ReferenceState

@dataclass
class ReferenceState:
    x_goal: float
    y_goal: float

    @classmethod
    def from_config(cls, config: ReferenceConfig):
        return cls(config.x_goal, config.y_goal)

@dataclass
class ControlCommand:
    timestamp: float
    tilt_about_x_deg: float
    tilt_about_y_deg: float
    servox_deg: float
    servoy_deg: float

    def get_cmd_str(self):
        return f"{self.servox_deg:.2f}, {self.servoy_deg:.2f}\n"

    def encode(self,encoding: str = "utf-8", errors: str = "strict") -> bytes:
        return f"{self.servox_deg:.2f}, {self.servoy_deg:.2f}\n".encode(encoding,errors)

