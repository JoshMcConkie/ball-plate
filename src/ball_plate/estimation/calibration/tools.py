'''
Tools to initiate uncertainty matrices (the Q in P^- = AP^A.T + Q)
'''

from dataclasses import dataclass
from time import time
import pandas as pd
from serial import Serial
from ball_plate import serial_io
from ball_plate.perception import ball, imu

import cv2

@dataclass
class CalibrationData:
    series: pd.DataFrame

           
def sample_ball_pos():
    pass






def calibrate_imu(feed: cv2.VideoCapture, 
                  ser,
                  estimate_process_noise: bool)->CalibrationData:
    
    # use previous calibration
    if not estimate_process_noise: 
        try:
            data = pd.read_csv("imu_data.csv")
        except FileNotFoundError as e:
            raise FileNotFoundError("No previous calibration file 'imu_data.csv' exists. " \
                                    "Please assign estimate_process_noise=False")
    # new calibration
    else:
        # Collect IMU samples
        history = []
        calibration_freq = 150
        sample_count = 1000
        t_last = 0.0
        i = 0
        while i < sample_count:
            t = time.monotonic()
            while t - t_last > 1 / calibration_freq:
                values = serial_io.fetch_values(ser)
                if values is not None:
                    t_last = time.monotonic()
                    history.append((t_last,*values))
                    i += 1
        data = pd.DataFrame(history, columns=["time","ax","ay",
                                              "az","gx","gy","gz"])
        diff = data.diff()
        diff_lag = diff.shift(1)

        gyro_noise_cov = data[["ax","ay","az",
                               "gx","gy","gz"]].cov()

        ''' Calibrating Q_B, as built in my Estimation Structure notes'''
        