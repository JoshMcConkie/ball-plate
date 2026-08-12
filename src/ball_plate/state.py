'''
state.py contains the state dataclasses for various measurements. The general flow is:

BallMeasurement → estimator → BallState
IMUReading      → estimator → TableState
SystemState     → controller → ControlCommand

'''
from dataclasses import dataclass
import numpy as np
from typing import ClassVar

from numpy.typing import NDArray

@dataclass
class IMUMeasurement:
    H: ClassVar[NDArray[np.float64]] = np.identity(4) # table state -> measurement state
                                                 
    timestamp: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    def get_meas_vector(self):
        return np.array([self.ax,self.ay,self.gx,self.gy,self.gz])

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
    timestamp: float
    confidence: float
    def get_meas_vector(self):
        return np.array([self.x_m,self.y_m])

@dataclass
class TableState:
    timestamp: float
    tilt_about_x: float
    tilt_about_y: float
    tilt_about_x_rate: float
    tilt_about_y_rate: float

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
    table: TableState
    ref_state: ReferenceState

@dataclass
class ReferenceState:
    timestamp: float
    x_goal: float
    y_goal: float

@dataclass
class ControlCommand:
    timestamp: float
    tilt_about_x_deg: float
    tilt_about_y_deg: float
    servox_deg: float
    servoy_deg: float

    def get_cmd_str(self):
        return f"{self.servox_deg:.2f}, {self.servoy_deg:.2f}\n"

