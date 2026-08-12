
from time import time

from serial import Serial
from ball_plate import serial_io
from ball_plate.state import IMUMeasurement

class IMUReader:
    def __init__(self, ser: Serial):
        self.ser = ser

    def measure(self)->IMUMeasurement | None:
        values = serial_io.fetch_values(self.ser)
        if values is None:
            return None
        return IMUMeasurement(time.monotonic(), *values)
    