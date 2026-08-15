
from time import time

import numpy as np

from ball_plate import serial_tools
from ball_plate.calibration import IMUCalibration
from ball_plate.state import IMUMeasurement


class IMUReader:
    def __init__(self, serial_io: serial_tools.SerialIO, imu_calibration: IMUCalibration):
        self.serial_io = serial_io
        self.alignment_matrix = imu_calibration.alignment_matrix

    def measure(self)->IMUMeasurement | None:
        meas = self.serial_io.fetch_values()
        if meas is None:
            return None
        values = np.asarray(meas).reshape(2,3)
        aligned_values = values @ self.alignment_matrix.T
        return IMUMeasurement(time.monotonic(), *aligned_values.ravel())
    