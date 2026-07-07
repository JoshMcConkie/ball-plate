'''
state.py contains the state dataclasses for various measurements. The general flow is:

BallMeasurement → estimator → BallState
IMUReading      → estimator → TableState
SystemState     → controller → ControlCommand

'''
from dataclasses import dataclass

@dataclass
class IMUReading:
    timestamp: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    
@dataclass
class BallMeasurement:
    timestamp: float
    x_px: int   # pixel coordinates
    y_px: int   
    x_m: int    # table coordinates
    y_m: int
    radius_px: int
    found: bool
    timestamp: float
    confidence: float

@dataclass
class TableState:
    timestamp: float
    roll: float
    pitch: float
    roll_rate: float
    pitch_rate: float

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
    roll_deg: float
    pitch_deg: float
    servox_deg: float
    servoy_deg: float

    def get_cmd_str(self):
        return f"{self.servox_deg:.2f}, {self.servoy_deg:.2f}\n"


