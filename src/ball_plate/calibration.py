'''
Tools to initiate uncertainty matrices (the Q in P^- = AP^A.T + Q)
'''

import json
from dataclasses import dataclass
from time import time

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from ball_plate.camera import Camera
from ball_plate.perception import ball


@dataclass(frozen=True)
class BallCalibration:
    meas_cov: np.ndarray  # measurement covariance (R)
    #TODO: calibrate acceleration variance
    acc_var: float = 0.1 # continuous acceleration variance (sigma_a)

    def save(self):
        data = {
            "version": 1,
            "meas_cov": self.meas_cov.tolist(),
            "acc_var": self.acc_var
        }
            
        with open("data/calibration/ball/calibration.json") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def load_calibration(cls):
        try:
            data = pd.read_csv("data/calibration/ball/calibration.json")
        except FileNotFoundError:
            raise FileNotFoundError("No previous calibration file 'data/calibration/ball/calibration.json' exists.")

class BallCalibrator:
    def __init__(self,camera: Camera, ball_detector: ball.BallDetector) -> None:
        self.feed = camera.feed
        self.ball_detector = ball_detector
        

    def collect_data(self)->pd.DataFrame:
        history = []
        calib_freq = 30
        sample_count = 1000
        t_last = 0.0
        i = 0
        while i < sample_count:
            ret, frame = self.feed.read()
            t = time.monotonic()
            while t - t_last > 1 / calib_freq:
                xy = self.ball_detector.measure_px_only(frame)
                if xy is not None:
                    t_last = time.monotonic()
                    history.append((t_last,*xy))
                    i += 1
        return pd.DataFrame(history, columns=["time","px","py"])
           
    def calibrate(self)->BallCalibration:
        assert self.ball_detector.map is not None
        ## Prompt user to place the ball before calibration
        while True:
            ret, frame = self.feed.read()
            if not ret:
                continue

            display = frame.copy()
            cv2.putText(
                display,
                "Place the ball on the table and make sure it is still.",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2,)
            cv2.putText(
                display,
                "Press SPACE when ready.",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0),2,)

            cv2.imshow("Camera Calibration", display)

            if cv2.waitKeyEx(1) == ord(" "):
                break
        cv2.destroyAllWindows()

        # Collect ball location samples in pixels
        data = self.collect_data()
        data.to_csv("data/calibration/ball/ball_cal_data.csv")

        data.dropna(inplace=True)
        data["x"] = (data["px"] - self.ball_detector.map.origin_px_x) * self.ball_detector.map.px_to_m_x
        data["y"] = (- data["py"] + self.ball_detector.map.origin_px_y) * self.ball_detector.map.px_to_m_y # invert y to increase upward
        
        meas_cov = (data[["x","y"]].cov()).to_numpy()    # measurement uncertainty (R)

        calibration = BallCalibration(meas_cov=meas_cov)
        calibration.save()
        
        return calibration


@dataclass(frozen=True)
class IMUCalibration:
    alignment_matrix: np.ndarray

    def save(self):
        data = {
            "version": 1,
            "alignment_matrix": self.alignment_matrix.tolist()
        }
        
        with open("data/calibration/imu/calibration.json") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def load_calibration(cls):
        try:
            data = pd.read_csv("data/calibration/imu/calibration.json")
        except FileNotFoundError:
            raise FileNotFoundError("No previous calibration file 'data/calibration/imu/calibration.json' exists.")


class IMUCalibrator:
    def __init__(self, controller, serial_io,
                 sample_freq=150,
                 sample_count=300,):
        self.controller = controller
        self.serial_io = serial_io
        self.sample_count = sample_count
        self.sample_freq = sample_freq
        
            
    def collect_data(self)->pd.DataFrame:
        history = []
        t_last = 0.0
        i = 0
        while i < self.sample_count:
            t = time.monotonic()
            while t - t_last > 1 / self.sample_freq:
                values = self.serial_io.fetch_values(self.serial_io)
                if values is not None:
                    t_last = time.monotonic()
                    history.append((t_last,*values))
                    i += 1
        return pd.DataFrame(history, columns=["time","ax","ay",
                                            "az","gx","gy","gz"]).dropna()

    def calibrate(self)->IMUCalibration:
        max_tilt_rad = np.deg2rad(self.controller.max_tilt_deg)
        expected_max_tilt_sin = np.sin(max_tilt_rad)
        expected_max_tilt_cos = np.cos(max_tilt_rad)

        # Collect samples
        self.controller.center_servos()
        time.sleep(1)
        center_data = self.collect_data()
        center_data.to_csv("data/calibration/imu/raw/level.csv")


        self.controller.xmax_ycenter()
        time.sleep(1)
        x_data = self.collect_data()
        x_data.to_csv("data/calibration/imu/raw/x_tilt.csv")

        self.controller.xcenter_ymax()
        time.sleep(1)
        y_data = self.collect_data()
        y_data.to_csv("data/calibration/imu/raw/y_tilt.csv")
        
        self.controller.center_servos()

        exp_cen_acc_unit_vector = np.array([0,0,1])
        exp_x_acc_unit_vector = np.array([0, expected_max_tilt_sin, - expected_max_tilt_cos])
        exp_y_acc_unit_vector = np.array([expected_max_tilt_sin,0, - expected_max_tilt_cos])

        imu_cen_acc_vector = center_data[["ax","ay","az"]].mean().to_numpy()
        imu_x_acc_vector = x_data[["ax","ay","az"]].mean().to_numpy()
        imu_y_acc_vector = y_data[["ax","ay","az"]].mean().to_numpy()

        imu_cen_acc_unit_vector = imu_cen_acc_vector / np.linalg.norm(imu_cen_acc_vector)
        imu_x_acc_unit_vector = imu_x_acc_vector / np.linalg.norm(imu_x_acc_vector)
        imu_y_acc_unit_vector = imu_y_acc_vector / np.linalg.norm(imu_y_acc_vector)
        
        rotation, error, *_ = Rotation.align_vectors([exp_cen_acc_unit_vector,exp_x_acc_unit_vector, exp_y_acc_unit_vector],
                                                     [imu_cen_acc_unit_vector,imu_x_acc_unit_vector, imu_y_acc_unit_vector])

        rot_matrix = rotation.as_matrix()
        np.linalg.det(rot_matrix)

        # check that the rotation matrix is valid
        assert np.isclose(np.linalg.det(rot_matrix),1.0)
        assert np.isclose(rot_matrix @ rot_matrix.T, np.eye(3))

        # rot_center_data = rotation.apply(center_data[["ax","ay","az"]].to_numpy())
        # rot_x_data = rotation.apply(x_data[["ax","ay","az"]].to_numpy())
        # rot_y_data = rotation.apply(y_data[["ax","ay","az"]].to_numpy())

        rot_cen_vector = rotation.apply(imu_cen_acc_unit_vector)
        rot_x_vector = rotation.apply(imu_x_acc_unit_vector)
        rot_y_vector = rotation.apply(imu_y_acc_unit_vector)

        assert np.isclose(rot_cen_vector,exp_cen_acc_unit_vector)
        assert np.isclose(rot_x_vector,exp_x_acc_unit_vector)
        assert np.isclose(rot_y_vector,exp_y_acc_unit_vector)

        calibration = IMUCalibration(
            alignment_matrix=rot_matrix
        )

        calibration.save()

        return calibration