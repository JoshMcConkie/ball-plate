import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class ControllerConfig:
    kp: float
    ki: float
    kd: float
    derivative_filter_alpha: float
    max_tilt_deg: float


@dataclass(frozen=True, slots=True)
class PlateConfig:
    width_m: float
    height_m: float


@dataclass(frozen=True, slots=True)
class SerialConfig:
    preferred_port: str
    baud_rate: int
    timeout_s: float

@dataclass(frozen=True, slots=True)
class DegreeToMicrosecondsConfig:
    slope_us_per_deg: float
    intercept_us: float


@dataclass(frozen=True, slots=True)
class PhysicalServoConfig:
    gpio_pin: int
    calibration_search_min_pulse_us: int
    calibration_search_max_pulse_us: int
    min_pulse_us: int
    max_pulse_us: int
    min_deg: float
    max_deg: float
    deg_to_us: DegreeToMicrosecondsConfig


@dataclass(frozen=True, slots=True)
class ServoConfig:
    arm_length_m: float
    center_deg: float
    a: PhysicalServoConfig
    b: PhysicalServoConfig

@dataclass(frozen=True, slots=True)
class CameraConfig:
    device_id: int
    target_ball_color_hsv: list[int]

@dataclass(frozen=True, slots=True)
class RuntimeRatesConfig:
    camera_capture: int
    imu_read: int
    state_estimation: int
    control: int
    servo_command: int
    debug_output: int


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    rates_hz: RuntimeRatesConfig

@dataclass(frozen=True,slots=True)
class ReferenceConfig:
    x_goal: float
    y_goal: float


@dataclass(frozen=True, slots=True)
class IMUConfig:
    firmware_stream_rate_hz: int


@dataclass(frozen=True, slots=True)
class CalibrationConfig:
    ball_load_path: Path
    imu_load_path: Path
    ball_save_path: Path
    imu_save_path: Path


@dataclass(frozen=True, slots=True)
class SystemConfig:
    controller: ControllerConfig
    plate: PlateConfig
    serial: SerialConfig
    servos: ServoConfig
    camera: CameraConfig
    reference: ReferenceConfig
    runtime: RuntimeConfig
    imu: IMUConfig
    calibration: CalibrationConfig

    


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "data"
    / "system_config.json"
)


class ServoCalibrationRequiredError(RuntimeError):
    """Raised when production code is loaded before servo calibration."""


def _load_physical_servo(
    servo_name: str,
    data: dict,
    *,
    center_deg: float,
) -> PhysicalServoConfig:
    required_calibration_fields = (
        "min_pulse_us",
        "max_pulse_us",
        "min_deg",
        "max_deg",
        "deg_to_us",
    )
    missing = [
        field
        for field in required_calibration_fields
        if data.get(field) is None
    ]
    if missing:
        raise ServoCalibrationRequiredError(
            f"Servo calibration required for servo {servo_name!r}: "
            f"missing {', '.join(missing)}"
        )

    map_data = data["deg_to_us"]
    search_min_us = data["calibration_search_min_pulse_us"]
    search_max_us = data["calibration_search_max_pulse_us"]
    min_us = data["min_pulse_us"]
    max_us = data["max_pulse_us"]
    min_deg = float(data["min_deg"])
    max_deg = float(data["max_deg"])
    slope = float(map_data["slope_us_per_deg"])
    intercept = float(map_data["intercept_us"])

    if not all(
        type(value) is int
        for value in (search_min_us, search_max_us, min_us, max_us)
    ):
        raise RuntimeError(
            f"Pulse bounds for servo {servo_name!r} must be integers"
        )
    if not search_min_us <= min_us < max_us <= search_max_us:
        raise RuntimeError(
            f"Calibration for servo {servo_name!r} is outside its "
            "configured search envelope"
        )
    if not (
        math.isfinite(min_deg)
        and math.isfinite(max_deg)
        and 0.0 <= min_deg < max_deg <= 180.0
    ):
        raise RuntimeError(
            f"Degree bounds for servo {servo_name!r} are invalid"
        )
    if not min_deg <= center_deg <= max_deg:
        raise RuntimeError(
            f"Servo center is outside servo {servo_name!r}'s calibrated range"
        )
    if not math.isfinite(slope) or slope == 0.0 or not math.isfinite(intercept):
        raise RuntimeError(
            f"Degree map for servo {servo_name!r} is invalid"
        )
    mapped_bounds = sorted(
        (slope * min_deg + intercept, slope * max_deg + intercept)
    )
    if not (
        math.isclose(mapped_bounds[0], min_us, abs_tol=0.5)
        and math.isclose(mapped_bounds[1], max_us, abs_tol=0.5)
    ):
        raise RuntimeError(
            f"Degree map for servo {servo_name!r} does not reconstruct "
            "its pulse bounds"
        )

    return PhysicalServoConfig(
        gpio_pin=data["gpio_pin"],
        calibration_search_min_pulse_us=search_min_us,
        calibration_search_max_pulse_us=search_max_us,
        min_pulse_us=min_us,
        max_pulse_us=max_us,
        min_deg=min_deg,
        max_deg=max_deg,
        deg_to_us=DegreeToMicrosecondsConfig(
            slope_us_per_deg=slope,
            intercept_us=intercept,
        ),
    )


def load_system_config(
    path: str | Path = DEFAULT_CONFIG_PATH,) -> SystemConfig:
    path = Path(path)

    try:
        with path.open(encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"System configuration not found: {path}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"Invalid JSON in {path}: line {exc.lineno}, "
            f"column {exc.colno}: {exc.msg}"
        ) from exc

    if data.get("schema_version") != 3:
        raise RuntimeError(
            f"Unsupported configuration schema version: "
            f"{data.get('schema_version')!r}"
        )

    try:
        servo_data = data["servos"]
        calibration_data = data["calibration"]

        center_deg = float(servo_data["center_deg"])
        servo_a = _load_physical_servo(
            "a", servo_data["a"], center_deg=center_deg
        )
        servo_b = _load_physical_servo(
            "b", servo_data["b"], center_deg=center_deg
        )
        return SystemConfig(
            controller=ControllerConfig(**data["controller"]),
            plate=PlateConfig(
                width_m=data["plate"]["width_m"],
                height_m=data["plate"]["height_m"],
            ),
            serial=SerialConfig(**data["serial"]),
            camera=CameraConfig(**data["camera"]),
            servos=ServoConfig(
                arm_length_m=data["servos"]["arm_length_m"],
                center_deg=center_deg,
                a=servo_a,
                b=servo_b,
            ),
            reference=ReferenceConfig(**data["reference"]),
            imu=IMUConfig(**data["imu"]),
            calibration=CalibrationConfig(
                ball_load_path=Path(calibration_data["ball_load_path"]),
                imu_load_path=Path(calibration_data["imu_load_path"]),
                ball_save_path=Path(calibration_data["ball_save_path"]),
                imu_save_path=Path(calibration_data["imu_save_path"]),
            ),
            runtime=RuntimeConfig(
                rates_hz=RuntimeRatesConfig(**data["runtime"]["rates_hz"])
            )
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"Invalid system configuration in {path}: {exc}"
        ) from exc
