from ball_plate.config import (
    ControllerConfig,
    DegreeToMicrosecondsConfig,
    PlateConfig,
    PhysicalServoConfig,
    ServoConfig,
)
from ball_plate.control import ServoController


def test_command_for_tilt_preserves_configured_tilt_bound():
    controller = ServoController(
        controller_config=ControllerConfig(
            kp=1.0,
            ki=1.0,
            kd=0.0,
            derivative_filter_alpha=0.5,
            max_tilt_deg=5.0,
        ),
        plate_config=PlateConfig(width_m=0.2, height_m=0.2),
        servo_config=_servo_config(),
    )

    command = controller.command_for_tilt(
        tilt_about_x_deg=20.0,
        tilt_about_y_deg=-20.0,
    )

    assert command.tilt_about_x_deg == controller.max_tilt_deg
    assert command.tilt_about_y_deg == -controller.max_tilt_deg
    assert controller.servo_config.a.min_deg <= command.servo_a_deg
    assert command.servo_a_deg <= controller.servo_config.a.max_deg
    assert controller.servo_config.b.min_deg <= command.servo_b_deg
    assert command.servo_b_deg <= controller.servo_config.b.max_deg


def _physical_servo_config(
    min_deg: float,
    max_deg: float,
) -> PhysicalServoConfig:
    return PhysicalServoConfig(
        gpio_pin=32,
        calibration_search_min_pulse_us=800,
        calibration_search_max_pulse_us=2200,
        min_pulse_us=900,
        max_pulse_us=2100,
        min_deg=min_deg,
        max_deg=max_deg,
        deg_to_us=DegreeToMicrosecondsConfig(
            slope_us_per_deg=10.0,
            intercept_us=600.0,
        ),
    )


def _servo_config() -> ServoConfig:
    return ServoConfig(
        arm_length_m=0.05,
        center_deg=90.0,
        a=_physical_servo_config(80.0, 100.0),
        b=_physical_servo_config(70.0, 110.0),
    )


def test_command_for_tilt_uses_independent_physical_servo_bounds():
    controller = ServoController(
        controller_config=ControllerConfig(
            kp=1.0,
            ki=1.0,
            kd=0.0,
            derivative_filter_alpha=0.5,
            max_tilt_deg=20.0,
        ),
        plate_config=PlateConfig(width_m=0.2, height_m=0.2),
        servo_config=_servo_config(),
    )

    command = controller.command_for_tilt(20.0, 20.0)

    assert command.servo_a_deg == 100.0
    assert command.servo_b_deg == 110.0
    assert controller.x_acceleration_servo is controller.servo_config.a
    assert controller.y_acceleration_servo is controller.servo_config.b
    assert command.encode() == b"100.00, 110.00\n"
