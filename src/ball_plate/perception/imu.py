
from serial import Serial
from ball_plate import serial_io
from ball_plate.state import IMUMeasurement


def measure(ser: Serial)->IMUMeasurement:
    return serial_io.fetch_packet(ser)