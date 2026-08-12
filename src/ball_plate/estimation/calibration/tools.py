'''
Tools to initiate uncertainty matrices (the Q in P^- = AP^A.T + Q)
'''

from dataclasses import dataclass
from time import time
import numpy as np
import pandas as pd
from serial import Serial
from ball_plate import serial_io
from ball_plate.perception import ball, imu

import cv2

@dataclass
class BallCalibration:
    meas_cov: np.ndarray  # continuous measurement covariance (R)
    acc_var: float # continuous acceleration variance (sigma_a)

           
def calibrate_ball(feed: cv2.VideoCapture,
                   ball_detector: ball.BallDetector,
                  estimate_process_noise: bool)->BallCalibration:
    # use previous calibration
    if not estimate_process_noise: 
        try:
            data = pd.read_csv("src/ball_plate/estimation/calibration/ball_cal_data.csv")
        except FileNotFoundError as e:
            raise FileNotFoundError("No previous calibration file 'ball_cal_data.csv' exists. " \
                                    "Please assign estimate_process_noise=False")
    # new calibration
    else:
        # Collect ball location samples in pixels
        history = []
        calib_freq = 30
        sample_count = 1000
        t_last = 0.0
        i = 0
        while i < sample_count:
            ret, frame = feed.read()
            t = time.monotonic()
            while t - t_last > 1 / calib_freq:
                xy = ball_detector.measure_px_only(frame)
                if xy is not None:
                    t_last = time.monotonic()
                    history.append((t_last,*xy))
                    i += 1
        data = pd.DataFrame(history, columns=["time","px","py"])
        data.to_csv("src/ball_plate/estimation/calibration/ball_cal_data.csv")

    data.dropna(inplace=True)
    data["x"] = (data["px"] - ball_detector.map.origin_px_x) * ball_detector.map.px_to_m_x
    data["y"] = (- data["py"] + ball_detector.map.origin_px_y) * ball_detector.map.px_to_m_y # invert y to increase upward

    calib_period = data["time"].diff().mean()
    
    meas_cov = (data[["x","y"]].cov()).to_numpy() / calib_period    # continuous measurement uncertainty (R)

    acc_var = 0.1 #TODO: sample for actual acc_var
    return BallCalibration(meas_cov=meas_cov, acc_var=acc_var)

""" Old IMU estimation
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
        """