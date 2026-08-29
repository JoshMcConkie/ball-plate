from __future__ import annotations

import hashlib
import json
import math
from typing import Any


class ConfigError(RuntimeError):
    pass


def _value_at(config: dict[str, Any], path: str) -> Any:
    value: Any = config
    for key in path.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ConfigError(f"Missing required configuration: {path}")
        value = value[key]
    return value


def _require_int(
    config: dict[str, Any],
    path: str,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    value = _value_at(config, path)
    if type(value) is not int:
        raise ConfigError(f"{path} must be an integer, got {value!r}")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{path} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ConfigError(f"{path} must be at most {maximum}")
    return value


def _require_number(
    config: dict[str, Any],
    path: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    value = _value_at(config, path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be numeric, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(f"{path} must be finite")
    if minimum is not None and number < minimum:
        raise ConfigError(f"{path} must be at least {minimum}")
    if maximum is not None and number > maximum:
        raise ConfigError(f"{path} must be at most {maximum}")
    return number


def calibration_values(config: dict[str, Any]) -> dict[str, Any]:
    if config.get("schema_version") != 3:
        raise ConfigError(
            "Servo calibration requires system_config.json schema_version 3"
        )

    values: dict[str, Any] = {
        "serial_baud": _require_int(config, "serial.baud_rate", minimum=1),
        "serial_timeout_s": _require_number(
            config, "serial.timeout_s", minimum=0.0
        ),
        "servo_center_deg": _require_number(
            config, "servos.center_deg", minimum=0.0, maximum=180.0
        ),
    }

    for servo_name in ("a", "b"):
        prefix = f"servos.{servo_name}"
        pin = _require_int(config, f"{prefix}.gpio_pin", minimum=0)
        search_min = _require_int(
            config,
            f"{prefix}.calibration_search_min_pulse_us",
            minimum=1,
        )
        search_max = _require_int(
            config,
            f"{prefix}.calibration_search_max_pulse_us",
            minimum=1,
        )
        if search_min >= search_max:
            raise ConfigError(
                f"{prefix}.calibration_search_min_pulse_us must be less "
                "than calibration_search_max_pulse_us"
            )
        values[f"servo_{servo_name}_pin"] = pin
        values[f"servo_{servo_name}_search_min_us"] = search_min
        values[f"servo_{servo_name}_search_max_us"] = search_max

    return values


def calibration_fingerprint(config: dict[str, Any]) -> str:
    encoded = json.dumps(
        calibration_values(config),
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(encoded).hexdigest()[:12]


def production_values(config: dict[str, Any]) -> dict[str, Any]:
    values = calibration_values(config)
    imu_rate_hz = _require_int(
        config,
        "imu.firmware_stream_rate_hz",
        minimum=1,
        maximum=1000,
    )
    if 1000 % imu_rate_hz != 0:
        raise ConfigError(
            "imu.firmware_stream_rate_hz must divide 1000 while main.cpp "
            "uses integer millisecond timing"
        )
    values["imu_rate_hz"] = imu_rate_hz

    center_deg = values["servo_center_deg"]
    for servo_name in ("a", "b"):
        prefix = f"servos.{servo_name}"
        missing = [
            key
            for key in (
                "min_pulse_us",
                "max_pulse_us",
                "min_deg",
                "max_deg",
                "deg_to_us",
            )
            if _value_at(config, f"{prefix}.{key}") is None
        ]
        if missing:
            raise ConfigError(
                f"Servo calibration required for servo {servo_name!r}: "
                f"missing {', '.join(missing)}"
            )

        min_us = _require_int(config, f"{prefix}.min_pulse_us", minimum=1)
        max_us = _require_int(config, f"{prefix}.max_pulse_us", minimum=1)
        min_deg = _require_number(
            config, f"{prefix}.min_deg", minimum=0.0, maximum=180.0
        )
        max_deg = _require_number(
            config, f"{prefix}.max_deg", minimum=0.0, maximum=180.0
        )
        slope = _require_number(
            config, f"{prefix}.deg_to_us.slope_us_per_deg"
        )
        intercept = _require_number(
            config, f"{prefix}.deg_to_us.intercept_us"
        )

        if min_us >= max_us:
            raise ConfigError(f"{prefix}.min_pulse_us must be less than max_pulse_us")
        if min_deg >= max_deg:
            raise ConfigError(f"{prefix}.min_deg must be less than max_deg")
        if not min_deg <= center_deg <= max_deg:
            raise ConfigError(
                f"servos.center_deg must be within servo {servo_name!r}'s calibrated range"
            )
        if not (
            values[f"servo_{servo_name}_search_min_us"]
            <= min_us
            < max_us
            <= values[f"servo_{servo_name}_search_max_us"]
        ):
            raise ConfigError(
                f"Calibrated pulse bounds for servo {servo_name!r} must remain "
                "inside its calibration search envelope"
            )
        if slope == 0.0:
            raise ConfigError(f"{prefix}.deg_to_us slope must not be zero")

        mapped_bounds = sorted(
            (slope * min_deg + intercept, slope * max_deg + intercept)
        )
        if not (
            math.isclose(mapped_bounds[0], min_us, abs_tol=0.5)
            and math.isclose(mapped_bounds[1], max_us, abs_tol=0.5)
        ):
            raise ConfigError(
                f"{prefix}.deg_to_us does not reconstruct its pulse bounds"
            )

        values[f"servo_{servo_name}_min_us"] = min_us
        values[f"servo_{servo_name}_max_us"] = max_us
        values[f"servo_{servo_name}_min_deg"] = min_deg
        values[f"servo_{servo_name}_max_deg"] = max_deg
        values[f"servo_{servo_name}_slope"] = slope
        values[f"servo_{servo_name}_intercept"] = intercept

    return values


def render_header(config: dict[str, Any], *, production: bool) -> str:
    values = production_values(config) if production else calibration_values(config)
    fingerprint = calibration_fingerprint(config)

    servo_blocks: list[str] = []
    for servo_name in ("a", "b"):
        lines = [
            f"namespace servo_{servo_name} {{",
            f"constexpr int gpio_pin = {values[f'servo_{servo_name}_pin']};",
            (
                "constexpr int calibration_search_min_pulse_us = "
                f"{values[f'servo_{servo_name}_search_min_us']};"
            ),
            (
                "constexpr int calibration_search_max_pulse_us = "
                f"{values[f'servo_{servo_name}_search_max_us']};"
            ),
        ]
        if production:
            lines.extend(
                [
                    f"constexpr int min_pulse_us = {values[f'servo_{servo_name}_min_us']};",
                    f"constexpr int max_pulse_us = {values[f'servo_{servo_name}_max_us']};",
                    f"constexpr double min_deg = {values[f'servo_{servo_name}_min_deg']!r};",
                    f"constexpr double max_deg = {values[f'servo_{servo_name}_max_deg']!r};",
                    (
                        "constexpr double deg_to_us_slope = "
                        f"{values[f'servo_{servo_name}_slope']!r};"
                    ),
                    (
                        "constexpr double deg_to_us_intercept = "
                        f"{values[f'servo_{servo_name}_intercept']!r};"
                    ),
                ]
            )
        lines.append("}")
        servo_blocks.append("\n".join(lines))

    imu_line = ""
    if production:
        imu_line = (
            "constexpr int imu_stream_rate_hz = "
            f"{values['imu_rate_hz']};\n"
        )

    return f"""\
#pragma once

namespace system_config {{

constexpr unsigned long serial_baud = {values['serial_baud']}UL;
constexpr double serial_timeout_s = {values['serial_timeout_s']!r};
constexpr double servo_center_deg = {values['servo_center_deg']!r};
{imu_line}
constexpr char calibration_fingerprint[] = "{fingerprint}";

{servo_blocks[0]}

{servo_blocks[1]}

}}  // namespace system_config
"""
