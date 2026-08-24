import json
from pathlib import Path

from ball_plate.config import (
    DEFAULT_CONFIG_PATH,
    CalibrationConfig,
    load_system_config,
)


def test_load_system_config_initializes_calibration_config():
    with DEFAULT_CONFIG_PATH.open(encoding="utf-8") as file:
        calibration_data = json.load(file)["calibration"]

    config = load_system_config()

    assert isinstance(config.calibration, CalibrationConfig)
    for field_name, configured_path in calibration_data.items():
        path = getattr(config.calibration, field_name)
        assert isinstance(path, Path)
        assert path == Path(configured_path)
