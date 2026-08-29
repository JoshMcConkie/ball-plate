from __future__ import annotations

import copy
import json
import math
import os
import stat
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from config_codegen import calibration_fingerprint, calibration_values


PROJECT_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_DIR.parent.parent
CONFIG_PATH = REPO_ROOT / "data" / "system_config.json"
AXIS_SEQUENCE = (
    ("x", "min_pulse"),
    ("x", "max_pulse"),
    ("y", "min_pulse"),
    ("y", "max_pulse"),
)


class CalibrationError(RuntimeError):
    pass


class CalibrationCancelled(CalibrationError):
    pass


class SerialConnection(Protocol):
    def write(self, data: bytes) -> int: ...

    def readline(self) -> bytes: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class AxisCalibration:
    min_pulse_us: int
    angle_at_min_pulse_us: float
    max_pulse_us: int
    angle_at_max_pulse_us: float
    min_deg: float
    max_deg: float
    slope_us_per_deg: float
    intercept_us: float


def lround(value: float) -> int:
    """Match C++ std::lround rather than Python's ties-to-even round."""
    if value >= 0.0:
        return math.floor(value + 0.5)
    return math.ceil(value - 0.5)


def map_degrees_to_microseconds(
    angle_deg: float,
    calibration: AxisCalibration,
) -> int:
    mapped = lround(
        calibration.slope_us_per_deg * angle_deg
        + calibration.intercept_us
    )
    return max(
        calibration.min_pulse_us,
        min(calibration.max_pulse_us, mapped),
    )


def calculate_axis_calibration(
    *,
    min_pulse_us: int,
    angle_at_min_pulse_us: float,
    max_pulse_us: int,
    angle_at_max_pulse_us: float,
    search_min_pulse_us: int,
    search_max_pulse_us: int,
    center_deg: float,
) -> AxisCalibration:
    angles = (angle_at_min_pulse_us, angle_at_max_pulse_us)
    if not all(math.isfinite(angle) for angle in angles):
        raise CalibrationError("Endpoint angles must be finite")
    if not all(0.0 <= angle <= 180.0 for angle in angles):
        raise CalibrationError("Endpoint angles must be within 0..180 degrees")
    if angle_at_min_pulse_us == angle_at_max_pulse_us:
        raise CalibrationError("Endpoint angles must differ")
    if min_pulse_us >= max_pulse_us:
        raise CalibrationError(
            "The max-pulse endpoint must exceed the min-pulse endpoint"
        )
    if not (
        search_min_pulse_us
        <= min_pulse_us
        < max_pulse_us
        <= search_max_pulse_us
    ):
        raise CalibrationError(
            "Measured pulse endpoints must remain inside the configured "
            "search envelope"
        )

    min_deg = min(angles)
    max_deg = max(angles)
    if not min_deg <= center_deg <= max_deg:
        raise CalibrationError(
            "Configured servos.center_deg must be inside the measured range"
        )

    slope = (max_pulse_us - min_pulse_us) / (
        angle_at_max_pulse_us - angle_at_min_pulse_us
    )
    intercept = min_pulse_us - slope * angle_at_min_pulse_us
    result = AxisCalibration(
        min_pulse_us=min_pulse_us,
        angle_at_min_pulse_us=angle_at_min_pulse_us,
        max_pulse_us=max_pulse_us,
        angle_at_max_pulse_us=angle_at_max_pulse_us,
        min_deg=min_deg,
        max_deg=max_deg,
        slope_us_per_deg=slope,
        intercept_us=intercept,
    )

    if (
        map_degrees_to_microseconds(angle_at_min_pulse_us, result)
        != min_pulse_us
        or map_degrees_to_microseconds(angle_at_max_pulse_us, result)
        != max_pulse_us
    ):
        raise CalibrationError("Calculated map does not reconstruct its endpoints")
    return result


def parse_result_line(line: str) -> tuple[str, AxisCalibration]:
    fields = line.split()
    if len(fields) != 10 or fields[0] != "RESULT" or fields[1] not in {"x", "y"}:
        raise CalibrationError(f"Malformed firmware result: {line!r}")
    try:
        calibration = AxisCalibration(
            min_pulse_us=int(fields[2]),
            angle_at_min_pulse_us=float(fields[3]),
            max_pulse_us=int(fields[4]),
            angle_at_max_pulse_us=float(fields[5]),
            min_deg=float(fields[6]),
            max_deg=float(fields[7]),
            slope_us_per_deg=float(fields[8]),
            intercept_us=float(fields[9]),
        )
    except ValueError as exc:
        raise CalibrationError(f"Malformed firmware result: {line!r}") from exc
    return fields[1], calibration


def validate_firmware_result(
    axis: str,
    reported: AxisCalibration,
    config: dict,
) -> AxisCalibration:
    axis_config = config["servos"]["axes"][axis]
    expected = calculate_axis_calibration(
        min_pulse_us=reported.min_pulse_us,
        angle_at_min_pulse_us=reported.angle_at_min_pulse_us,
        max_pulse_us=reported.max_pulse_us,
        angle_at_max_pulse_us=reported.angle_at_max_pulse_us,
        search_min_pulse_us=axis_config[
            "calibration_search_min_pulse_us"
        ],
        search_max_pulse_us=axis_config[
            "calibration_search_max_pulse_us"
        ],
        center_deg=config["servos"]["center_deg"],
    )
    for field in (
        "min_deg",
        "max_deg",
        "slope_us_per_deg",
        "intercept_us",
    ):
        if not math.isclose(
            getattr(reported, field),
            getattr(expected, field),
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise CalibrationError(
                f"Firmware and host calculations disagree for axis {axis!r}"
            )
    return expected


def apply_calibrations(
    config: dict,
    calibrations: dict[str, AxisCalibration],
) -> dict:
    if set(calibrations) != {"x", "y"}:
        raise CalibrationError("Both X and Y calibration results are required")

    updated = copy.deepcopy(config)
    for axis in ("x", "y"):
        result = calibrations[axis]
        axis_config = updated["servos"]["axes"][axis]
        axis_config.update(
            {
                "min_pulse_us": result.min_pulse_us,
                "max_pulse_us": result.max_pulse_us,
                "min_deg": round(result.min_deg, 9),
                "max_deg": round(result.max_deg, 9),
                "deg_to_us": {
                    "slope_us_per_deg": round(
                        result.slope_us_per_deg, 9
                    ),
                    "intercept_us": round(result.intercept_us, 9),
                },
            }
        )
    return updated


def atomic_update_config(
    path: Path,
    *,
    expected_bytes: bytes,
    updated_config: dict,
) -> None:
    if path.read_bytes() != expected_bytes:
        raise CalibrationError(
            "system_config.json changed during calibration; no values were written"
        )

    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temp_path = Path(temporary.name)
            json.dump(updated_config, temporary, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temp_path, stat.S_IMODE(path.stat().st_mode))
        os.replace(temp_path, path)
        temp_path = None
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def _read_line(connection: SerialConnection, *, deadline: float) -> str:
    while time.monotonic() < deadline:
        raw = connection.readline()
        if raw:
            return raw.decode("utf-8", errors="replace").strip()
    raise CalibrationError("Timed out waiting for calibration firmware")


def wait_for_ready(
    connection: SerialConnection,
    *,
    expected_fingerprint: str,
    timeout_s: float = 8.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while True:
        line = _read_line(connection, deadline=deadline)
        if not line.startswith("READY "):
            continue
        actual = line.removeprefix("READY ").strip()
        if actual != expected_fingerprint:
            raise CalibrationError(
                "Calibration firmware/config fingerprint mismatch: "
                f"firmware={actual}, config={expected_fingerprint}. "
                "Rebuild and manually upload the calibration environment."
            )
        return


def _send(connection: SerialConnection, command: str) -> None:
    connection.write(f"{command}\n".encode())
    connection.flush()


def _exchange(
    connection: SerialConnection,
    command: str,
    *,
    terminal_prefixes: tuple[str, ...],
    output: Callable[[str], None],
    timeout_s: float = 5.0,
) -> list[str]:
    _send(connection, command)
    deadline = time.monotonic() + timeout_s
    lines: list[str] = []
    while True:
        line = _read_line(connection, deadline=deadline)
        if not line:
            continue
        output(f"device> {line}")
        lines.append(line)
        if line.startswith("ERROR ") or line.startswith(terminal_prefixes):
            return lines


def _require_success(lines: list[str]) -> None:
    error = next((line for line in lines if line.startswith("ERROR ")), None)
    if error is not None:
        raise CalibrationError(error)


def _prompt_for_endpoint(
    connection: SerialConnection,
    *,
    axis: str,
    endpoint: str,
    final_endpoint: bool,
    input_fn: Callable[[str], str],
    output: Callable[[str], None],
) -> list[str]:
    output(
        f"\nAxis {axis.upper()} {endpoint.replace('_', ' ')}: jog to the "
        "desired pose, then capture its measured arm angle."
    )
    output("Commands: jog <signed_us>, set <pulse_us>, status, capture, abort")
    while True:
        command = input_fn("calibrate> ").strip()
        if command == "abort":
            _exchange(
                connection,
                "ABORT",
                terminal_prefixes=("ACK DISARM",),
                output=output,
            )
            raise CalibrationCancelled("Calibration aborted by user")
        if command == "status":
            _exchange(
                connection,
                "STATUS",
                terminal_prefixes=("PROMPT ",),
                output=output,
            )
            continue
        if command.startswith("jog ") or command.startswith("set "):
            verb, value = command.split(maxsplit=1)
            lines = _exchange(
                connection,
                f"{verb.upper()} {value}",
                terminal_prefixes=("STATE ",),
                output=output,
            )
            if any(line.startswith("ERROR ") for line in lines):
                continue
            continue
        if command == "capture":
            angle_text = input_fn("Measured arm angle in degrees: ").strip()
            lines = _exchange(
                connection,
                f"CAPTURE {angle_text}",
                terminal_prefixes=("COMPLETE",) if final_endpoint else ("PROMPT ",),
                output=output,
            )
            if any(line.startswith("ERROR ") for line in lines):
                continue
            return lines
        output("Unknown command.")


def run_session(
    connection: SerialConnection,
    config: dict,
    *,
    input_fn: Callable[[str], str] = input,
    output: Callable[[str], None] = print,
) -> dict[str, AxisCalibration]:
    expected_fingerprint = calibration_fingerprint(config)
    wait_for_ready(connection, expected_fingerprint=expected_fingerprint)
    output(f"Connected to matching calibration firmware ({expected_fingerprint}).")

    confirmation = input_fn(
        "Clear the mechanism workspace, keep the rig attended, and type ARM: "
    ).strip()
    if confirmation != "ARM":
        raise CalibrationCancelled("Servos were not armed")

    armed = False
    result_lines: list[str] = []
    try:
        lines = _exchange(
            connection,
            "ARM",
            terminal_prefixes=("PROMPT ",),
            output=output,
        )
        _require_success(lines)
        armed = True

        for index, (axis, endpoint) in enumerate(AXIS_SEQUENCE):
            result_lines.extend(
                _prompt_for_endpoint(
                    connection,
                    axis=axis,
                    endpoint=endpoint,
                    final_endpoint=index == len(AXIS_SEQUENCE) - 1,
                    input_fn=input_fn,
                    output=output,
                )
            )

        disarm_confirmation = input_fn(
            "Servos are centered. Type DISARM to release them: "
        ).strip()
        if disarm_confirmation != "DISARM":
            raise CalibrationCancelled("Explicit disarm confirmation was not entered")
        lines = _exchange(
            connection,
            "DISARM",
            terminal_prefixes=("ACK DISARM",),
            output=output,
        )
        _require_success(lines)
        armed = False
    finally:
        if armed:
            try:
                _send(connection, "ABORT")
            except Exception:
                pass

    reported = dict(
        parse_result_line(line)
        for line in result_lines
        if line.startswith("RESULT ")
    )
    if set(reported) != {"x", "y"}:
        raise CalibrationError("Firmware did not return both axis results")
    return {
        axis: validate_firmware_result(axis, reported[axis], config)
        for axis in ("x", "y")
    }


def main() -> int:
    original_bytes = CONFIG_PATH.read_bytes()
    try:
        config = json.loads(original_bytes)
        values = calibration_values(config)
    except (json.JSONDecodeError, KeyError, RuntimeError) as exc:
        print(f"Configuration error: {exc}")
        return 2

    try:
        import serial

        connection = serial.Serial(
            port=config["serial"]["preferred_port"],
            baudrate=values["serial_baud"],
            timeout=values["serial_timeout_s"],
        )
    except Exception as exc:
        print(f"Could not open configured serial port: {exc}")
        return 2

    try:
        calibrations = run_session(connection, config)
    except (CalibrationError, KeyboardInterrupt) as exc:
        print(f"Calibration stopped: {exc}")
        return 1
    finally:
        connection.close()

    updated = apply_calibrations(config, calibrations)
    print("\nProposed servos configuration:")
    print(json.dumps(updated["servos"], indent=2))
    if input("Type WRITE to update data/system_config.json: ").strip() != "WRITE":
        print("Configuration was not changed.")
        return 1

    try:
        atomic_update_config(
            CONFIG_PATH,
            expected_bytes=original_bytes,
            updated_config=updated,
        )
    except (CalibrationError, OSError) as exc:
        print(f"Configuration was not changed: {exc}")
        return 1

    print(f"Updated {CONFIG_PATH}")
    print("Rebuild and manually upload the esp32dev production environment.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
