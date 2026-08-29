import copy
import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "firmware" / "260530-200235-esp32dev" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from config_codegen import ConfigError, calibration_fingerprint, render_header


spec = importlib.util.spec_from_file_location(
    "calibrate_servos", SCRIPTS / "calibrate_servos.py"
)
calibrate_servos = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = calibrate_servos
spec.loader.exec_module(calibrate_servos)


AxisCalibration = calibrate_servos.AxisCalibration
CalibrationCancelled = calibrate_servos.CalibrationCancelled
CalibrationError = calibrate_servos.CalibrationError


def load_config() -> dict:
    return json.loads((ROOT / "data" / "system_config.json").read_text())


def result_for(
    *, min_us=900, low_angle=30.0, max_us=2100, high_angle=150.0
):
    return calibrate_servos.calculate_axis_calibration(
        min_pulse_us=min_us,
        angle_at_min_pulse_us=low_angle,
        max_pulse_us=max_us,
        angle_at_max_pulse_us=high_angle,
        search_min_pulse_us=800,
        search_max_pulse_us=2200,
        center_deg=90.0,
    )


def test_positive_slope_map_reconstructs_endpoints_and_rounds_like_cpp():
    result = result_for()

    assert result.slope_us_per_deg == 10.0
    assert result.intercept_us == 600.0
    assert calibrate_servos.map_degrees_to_microseconds(30.0, result) == 900
    assert calibrate_servos.lround(1000.5) == 1001


def test_negative_slope_preserves_endpoint_pairing_and_numeric_degree_bounds():
    result = result_for(low_angle=150.0, high_angle=30.0)

    assert result.min_deg == 30.0
    assert result.max_deg == 150.0
    assert result.slope_us_per_deg == -10.0
    assert result.intercept_us == 2400.0
    assert calibrate_servos.map_degrees_to_microseconds(150.0, result) == 900
    assert calibrate_servos.map_degrees_to_microseconds(30.0, result) == 2100


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"angle_at_max_pulse_us": 30.0}, "angles must differ"),
        ({"angle_at_min_pulse_us": -1.0}, "0..180"),
        ({"max_pulse_us": 900}, "must exceed"),
        ({"min_pulse_us": 700}, "search envelope"),
        (
            {
                "angle_at_min_pulse_us": 10.0,
                "angle_at_max_pulse_us": 80.0,
            },
            "center_deg",
        ),
    ],
)
def test_invalid_calibration_is_rejected(overrides, message):
    arguments = {
        "min_pulse_us": 900,
        "angle_at_min_pulse_us": 30.0,
        "max_pulse_us": 2100,
        "angle_at_max_pulse_us": 150.0,
        "search_min_pulse_us": 800,
        "search_max_pulse_us": 2200,
        "center_deg": 90.0,
    }
    arguments.update(overrides)

    with pytest.raises(CalibrationError, match=message):
        calibrate_servos.calculate_axis_calibration(**arguments)


def test_apply_calibrations_preserves_unrelated_config():
    config = load_config()
    config["unrelated"] = {"preserve": True}
    updated = calibrate_servos.apply_calibrations(
        config,
        {"x": result_for(), "y": result_for(low_angle=150, high_angle=30)},
    )

    assert updated["unrelated"] == {"preserve": True}
    assert config["servos"]["axes"]["x"]["deg_to_us"] is None
    assert updated["servos"]["axes"]["y"]["deg_to_us"] == {
        "slope_us_per_deg": -10.0,
        "intercept_us": 2400.0,
    }


def test_atomic_update_rejects_concurrent_config_change(tmp_path):
    path = tmp_path / "system_config.json"
    original = b'{"value": 1}\n'
    path.write_bytes(original)
    path.write_text('{"value": 2}\n')

    with pytest.raises(CalibrationError, match="changed during calibration"):
        calibrate_servos.atomic_update_config(
            path,
            expected_bytes=original,
            updated_config={"value": 3},
        )

    assert json.loads(path.read_text()) == {"value": 2}


def test_atomic_update_writes_complete_config_and_preserves_file_mode(tmp_path):
    path = tmp_path / "system_config.json"
    original = b'{"value": 1}\n'
    path.write_bytes(original)
    path.chmod(0o640)

    calibrate_servos.atomic_update_config(
        path,
        expected_bytes=original,
        updated_config={"value": 2, "other": True},
    )

    assert json.loads(path.read_text()) == {"value": 2, "other": True}
    assert path.stat().st_mode & 0o777 == 0o640


def test_atomic_update_cleans_up_after_replace_failure(tmp_path, monkeypatch):
    path = tmp_path / "system_config.json"
    original = b'{"value": 1}\n'
    path.write_bytes(original)

    def fail_replace(source, destination):
        raise OSError("replace failed")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failed"):
        calibrate_servos.atomic_update_config(
            path,
            expected_bytes=original,
            updated_config={"value": 2},
        )

    assert path.read_bytes() == original
    assert list(tmp_path.glob(".system_config.json.*.tmp")) == []


class FakeSerial:
    def __init__(self, ready_fingerprint: str, *, arm_error: bool = False):
        self.lines = [f"READY {ready_fingerprint}".encode() + b"\n"]
        self.commands = []
        self.capture_count = 0
        self.arm_error = arm_error

    def write(self, data: bytes) -> int:
        command = data.decode().strip()
        self.commands.append(command)
        if command == "ARM":
            if self.arm_error:
                self.lines.append(b"ERROR STATE cannot arm\n")
            else:
                self.lines.extend(
                    [b"ACK ARM\n", b"STATE X_LOW x 1500 y 1500\n", b"PROMPT x min_pulse\n"]
                )
        elif command.startswith("SET "):
            self.lines.extend([b"ACK SET\n", b"STATE MOVED x 900 y 1500\n"])
        elif command.startswith("CAPTURE "):
            responses = (
                [b"STATE X_HIGH x 900 y 1500\n", b"PROMPT x max_pulse\n"],
                [b"STATE Y_LOW x 2100 y 1500\n", b"PROMPT y min_pulse\n"],
                [b"STATE Y_HIGH x 2100 y 900\n", b"PROMPT y max_pulse\n"],
                [
                    b"RESULT x 900 30.000000000 2100 150.000000000 30.000000000 150.000000000 10.000000000 600.000000000\n",
                    b"RESULT y 900 150.000000000 2100 30.000000000 30.000000000 150.000000000 -10.000000000 2400.000000000\n",
                    b"COMPLETE\n",
                ],
            )
            self.lines.extend(responses[self.capture_count])
            self.capture_count += 1
        elif command in {"DISARM", "ABORT"}:
            self.lines.append(b"ACK DISARM\n")
        return len(data)

    def readline(self) -> bytes:
        return self.lines.pop(0) if self.lines else b""

    def flush(self):
        pass

    def close(self):
        pass


def input_sequence(*answers):
    iterator = iter(answers)
    return lambda prompt: next(iterator)


def test_run_session_collects_and_verifies_both_axes():
    config = load_config()
    connection = FakeSerial(calibration_fingerprint(config))
    input_fn = input_sequence(
        "ARM",
        "set 900", "capture", "30",
        "set 2100", "capture", "150",
        "set 900", "capture", "150",
        "set 2100", "capture", "30",
        "DISARM",
    )

    results = calibrate_servos.run_session(
        connection, config, input_fn=input_fn, output=lambda line: None
    )

    assert results["x"].slope_us_per_deg == 10.0
    assert results["y"].slope_us_per_deg == -10.0
    assert connection.commands[-1] == "DISARM"


def test_run_session_rejects_fingerprint_mismatch():
    config = load_config()
    connection = FakeSerial("stale-config")

    with pytest.raises(CalibrationError, match="fingerprint mismatch"):
        calibrate_servos.run_session(connection, config)

    assert connection.commands == []


def test_wait_for_ready_reports_serial_timeout():
    connection = FakeSerial("unused")
    connection.lines.clear()

    with pytest.raises(CalibrationError, match="Timed out"):
        calibrate_servos.wait_for_ready(
            connection,
            expected_fingerprint="expected",
            timeout_s=0.001,
        )


def test_run_session_cancellation_before_arm_does_not_command_hardware():
    config = load_config()
    connection = FakeSerial(calibration_fingerprint(config))

    with pytest.raises(CalibrationCancelled):
        calibrate_servos.run_session(
            connection,
            config,
            input_fn=input_sequence("cancel"),
            output=lambda line: None,
        )

    assert connection.commands == []


def test_run_session_device_error_attempts_abort():
    config = load_config()
    connection = FakeSerial(calibration_fingerprint(config), arm_error=True)

    with pytest.raises(CalibrationError, match="cannot arm"):
        calibrate_servos.run_session(
            connection,
            config,
            input_fn=input_sequence("ARM"),
            output=lambda line: None,
        )

    assert connection.commands == ["ARM"]


def test_codegen_allows_calibration_but_blocks_production_until_complete():
    config = load_config()

    calibration_header = render_header(config, production=False)
    assert "calibration_search_min_pulse_us" in calibration_header
    with pytest.raises(ConfigError, match="Servo calibration required"):
        render_header(config, production=True)


def test_codegen_renders_verified_production_map():
    config = load_config()
    updated = calibrate_servos.apply_calibrations(
        config,
        {"x": result_for(), "y": result_for(low_angle=150, high_angle=30)},
    )

    header = render_header(updated, production=True)

    assert "constexpr double deg_to_us_slope = 10.0;" in header
    assert "constexpr double deg_to_us_slope = -10.0;" in header
