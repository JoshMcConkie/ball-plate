import json
import copy
from pathlib import Path

import pytest

from ball_plate.config import (
    DEFAULT_CONFIG_PATH,
    CalibrationConfig,
    ServoCalibrationRequiredError,
    load_system_config,
)


def _calibrated_config() -> dict:
    with DEFAULT_CONFIG_PATH.open(encoding="utf-8") as file:
        data = copy.deepcopy(json.load(file))
    for axis, slope, intercept in (
        ("x", 10.0, 600.0),
        ("y", -10.0, 2400.0),
    ):
        axis_data = data["servos"]["axes"][axis]
        axis_data.update(
            {
                "min_pulse_us": 900,
                "max_pulse_us": 2100,
                "min_deg": 30.0,
                "max_deg": 150.0,
                "deg_to_us": {
                    "slope_us_per_deg": slope,
                    "intercept_us": intercept,
                },
            }
        )
    return data


def test_default_config_requires_servo_calibration():
    with pytest.raises(ServoCalibrationRequiredError, match="axis 'x'"):
        load_system_config()


def test_load_system_config_initializes_calibration_config(tmp_path):
    data = _calibrated_config()
    config_path = tmp_path / "system_config.json"
    config_path.write_text(json.dumps(data), encoding="utf-8")
    calibration_data = data["calibration"]

    config = load_system_config(config_path)

    assert isinstance(config.calibration, CalibrationConfig)
    for field_name, configured_path in calibration_data.items():
        path = getattr(config.calibration, field_name)
        assert isinstance(path, Path)
        assert path == Path(configured_path)

    assert config.servos.axes.x.min_deg == 30.0
    assert config.servos.axes.y.deg_to_us.slope_us_per_deg == -10.0
