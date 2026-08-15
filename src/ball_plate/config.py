from ball_plate.state import ReferenceState

BAUD_RATE = 115200
SERIAL_PORT = '/dev/ttyUSB0'

CAM_ID = 0

REFERENCE_STATE = ReferenceState(timestamp=0.0,x_goal=0.0,y_goal=0.0)
CAMERA_HZ = 60
IMU_HZ = 100
STATE_EST_HZ = 100
CONTROL_HZ = 50
SERIAL_HZ = 50
DEBUG_HZ = 5

BALL_COLOR = (0, 160, 255)

TABLE_W_M = 0.22  # table width in meters (x)
TABLE_H_M = 0.22  # table height in meters (y)

SERVO_ARM_LENGTH = 0.023  # servo arm length in meters
SERVO_CENTER_DEG = 90.0   # servo angle that holds the plate flat (firmware neutral)

IMU_SEND_RATE = 200 # Hz. Matches value in esp32dev/src/main.cpp


from dataclasses import dataclass
from pathlib import Path
import json


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
class ServoConfig:
    arm_length_m: float
    center_deg: float
    min_deg: float
    max_deg: float

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
class SystemConfig:
    controller: ControllerConfig
    plate: PlateConfig
    serial: SerialConfig
    servos: ServoConfig
    camera: CameraConfig
    reference: ReferenceConfig
    runtime: RuntimeConfig

    


DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "data"
    / "system_config.json"
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

    if data.get("schema_version") != 1:
        raise RuntimeError(
            f"Unsupported configuration schema version: "
            f"{data.get('schema_version')!r}"
        )

    try:
        servo_data = data["servos"]

        ServoConfig(
            arm_length_m=servo_data["arm_length_m"],
            center_deg=servo_data["center_deg"],
            min_deg=servo_data["min_deg"],
            max_deg=servo_data["max_deg"],
        )
        return SystemConfig(
            controller=ControllerConfig(**data["controller"]),
            plate=PlateConfig(**data["plate"]),
            serial=SerialConfig(**data["serial"]),
            camera=CameraConfig(**data["camera"]),
            servos=ServoConfig(
                arm_length_m=data["servos"]["arm_length_m"],
                center_deg=data["servos"]["center_deg"],
                min_deg=data["servos"]["min_deg"],
                max_deg=data["servos"]["max_deg"],
            ),
            reference=ReferenceConfig(**data["reference"]),
            runtime=RuntimeConfig(
                rates_hz=RuntimeRatesConfig(**data["runtime"]["rates_hz"])
            )
        )
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Invalid system configuration in {path}: {exc}"
        ) from exc