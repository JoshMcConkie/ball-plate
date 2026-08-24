'''
Tools to initiate uncertainty matrices (the Q in P^- = AP^A.T + Q)
'''

import json
from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from ball_plate.camera import Camera
from ball_plate.control import ServoController
from ball_plate.perception import ball
from ball_plate.serial_tools import SerialIO


@dataclass(frozen=True)
class BallCalibration:
    meas_cov: np.ndarray  # measurement covariance (R)
    #TODO: calibrate acceleration variance
    acc_var: float = 0.1 # continuous acceleration variance (sigma_a)

    def save(self, save_path: str | Path) -> None:
        save_path = Path(save_path)
        data = {
            "version": 1,
            "meas_cov": self.meas_cov.tolist(),
            "acc_var": self.acc_var
        }
            
        with save_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def load(
        cls,
        load_path: str | Path,
    ) -> "BallCalibration":
        load_path = Path(load_path)

        try:
            with load_path.open(encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"Ball calibration not found: {load_path}"
            ) from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Invalid JSON in {load_path}: line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc

        if data.get("version") != 1:
            raise RuntimeError(
                f"Unsupported ball calibration version: "
                f"{data.get('version')!r}"
            )

        try:
            return cls(
                meas_cov=np.asarray(data["meas_cov"], dtype=float),
                acc_var=data["acc_var"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"Invalid ball calibration in {load_path}: {exc}"
            ) from exc

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
           
    def calibrate(self, save_path: str | Path) -> BallCalibration:
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
        calibration.save(save_path)
        
        return calibration


@dataclass(frozen=True)
class IMUCalibration:
    alignment_matrix: np.ndarray

    def save(self, save_path: str | Path) -> None:
        save_path = Path(save_path)
        data = {
            "version": 1,
            "alignment_matrix": self.alignment_matrix.tolist()
        }
        
        with save_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
        print(f"Saved calibration to {save_path}")

    @classmethod
    def _load(
        cls,
        load_path: str | Path,
    ) -> "IMUCalibration":
        load_path = Path(load_path)

        print(f"Loading Calibration at {load_path}")

        try:
            with load_path.open(encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"IMU calibration not found: {load_path}"
            ) from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Invalid JSON in {load_path}: line {exc.lineno}, "
                f"column {exc.colno}: {exc.msg}"
            ) from exc

        if data.get("version") != 1:
            raise RuntimeError(
                f"Unsupported IMU calibration version: "
                f"{data.get('version')!r}"
            )

        try:
            return cls(
                alignment_matrix=np.asarray(
                    data["alignment_matrix"], dtype=float
                )
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"Invalid IMU calibration in {load_path}: {exc}"
            ) from exc

    @classmethod
    def load(
        cls,
        load_path: str | Path,
    ) -> "IMUCalibration":
        load_path = Path(load_path)

        print(f"Loading Calibration at {load_path}")
        calibration = cls._load(load_path)
        print("Calibration loaded.")

        return calibration
        
class IMUCalibrator:
    def __init__(self, controller, serial_io,
                 sample_freq=150,
                 sample_count=300,):
        self.controller: ServoController = controller
        self.serial_io: SerialIO = serial_io
        self.sample_count: int = sample_count
        self.sample_freq: float = sample_freq
        
            
    def collect_data(self)->pd.DataFrame:
        history = []
        t_last = 0.0
        i = 0
        while i < self.sample_count:
            t = time.monotonic()
            while t - t_last > 1 / self.sample_freq:
                values = self.serial_io.fetch_values()
                if values is not None:
                    t_last = time.monotonic()
                    history.append((t_last,*values))
                    i += 1
        return pd.DataFrame(history, columns=["time","ax","ay",
                                            "az","gx","gy","gz"]).dropna()

    def calibrate(self, save_path: str | Path) -> IMUCalibration:
        print("Calibrating IMU...")

        max_tilt_rad = np.deg2rad(self.controller.max_tilt_deg)
        expected_max_tilt_sin = np.sin(max_tilt_rad)
        expected_max_tilt_cos = np.cos(max_tilt_rad)

        max_xy_tilt_cmds = self.controller.max_tilt_cmds

        # Collect samples at center position
        print(
            "   Collecting samples at: "
            f"x={max_xy_tilt_cmds[0].tilt_about_x_deg:.2f} deg, "
            f"y={max_xy_tilt_cmds[0].tilt_about_y_deg:.2f} deg"
        )
        center_cmd = self.controller.center_cmd
        self.serial_io.send_packet(center_cmd)
        time.sleep(1)
        center_data = self.collect_data()
        center_data.to_csv("data/calibration/imu/raw/level.csv")

        # Collect samples at x position
        print(
            "   Collecting samples at: "
            f"x={max_xy_tilt_cmds[0].tilt_about_x_deg:.2f} deg, "
            f"y={max_xy_tilt_cmds[0].tilt_about_y_deg:.2f} deg"
        )

        self.serial_io.send_packet(max_xy_tilt_cmds[0])
        time.sleep(1)
        x_data = self.collect_data()
        x_data.to_csv("data/calibration/imu/raw/x_tilt.csv")

        # Collect samples at x position
        print(
            "   Collecting samples at: "
            f"x={max_xy_tilt_cmds[1].tilt_about_x_deg:.2f} deg, "
            f"y={max_xy_tilt_cmds[1].tilt_about_y_deg:.2f} deg"
        )
        self.serial_io.send_packet(max_xy_tilt_cmds[1])
        time.sleep(1)
        y_data = self.collect_data()
        y_data.to_csv("data/calibration/imu/raw/y_tilt.csv")
        
        self.serial_io.send_packet(center_cmd)

        exp_cen_acc_unit_vector = np.array([0.0,0.0,-1.0])
        exp_x_acc_unit_vector = np.array([0, expected_max_tilt_sin, - expected_max_tilt_cos])
        exp_y_acc_unit_vector = np.array([expected_max_tilt_sin,0, - expected_max_tilt_cos])

        print(center_data.describe())
        print(x_data.describe())
        print(y_data.describe())

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
        np.testing.assert_allclose(
            np.linalg.det(rot_matrix),
            1.0,
            rtol=1e-5,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            rot_matrix @ rot_matrix.T,
            np.eye(3),
            rtol=1e-5,
            atol=1e-8,
        )

        rot_cen_vector = rotation.apply(imu_cen_acc_unit_vector)
        rot_x_vector = rotation.apply(imu_x_acc_unit_vector)
        rot_y_vector = rotation.apply(imu_y_acc_unit_vector)

        # assert the final calibrated force vector is close to the expected.
        np.testing.assert_allclose(
            rot_cen_vector,
            exp_cen_acc_unit_vector,
            rtol=1e-5,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            rot_x_vector,
            exp_x_acc_unit_vector,
            rtol=1e-5,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            rot_y_vector,
            exp_y_acc_unit_vector,
            rtol=1e-5,
            atol=1e-8,
        )

        calibration = IMUCalibration(
            alignment_matrix=rot_matrix
        )

        calibration.save(save_path)
        print("Calibration finished.")

        return calibration
