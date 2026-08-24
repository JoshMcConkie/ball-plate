import numpy as np

from ball_plate.calibration import BallCalibration, IMUCalibration


def test_ball_calibration_round_trips_at_supplied_path(tmp_path):
    path = tmp_path / "ball-calibration.json"
    expected = BallCalibration(
        meas_cov=np.array([[0.1, 0.01], [0.01, 0.2]]),
        acc_var=0.3,
    )
    expected.save(path)

    calibration = BallCalibration.load(path)

    np.testing.assert_array_equal(calibration.meas_cov, expected.meas_cov)
    assert calibration.acc_var == expected.acc_var


def test_imu_calibration_round_trips_at_supplied_path(tmp_path):
    path = tmp_path / "imu-calibration.json"
    expected = IMUCalibration(alignment_matrix=np.eye(3))
    expected.save(path)

    calibration = IMUCalibration.load(path)

    np.testing.assert_array_equal(
        calibration.alignment_matrix,
        expected.alignment_matrix,
    )
