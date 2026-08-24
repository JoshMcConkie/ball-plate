from ball_plate.config import ControllerConfig, PlateConfig, ServoConfig
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
        servo_config=ServoConfig(
            arm_length_m=0.05,
            center_deg=90.0,
            min_deg=30.0,
            max_deg=150.0,
        ),
    )

    command = controller.command_for_tilt(
        tilt_about_x_deg=20.0,
        tilt_about_y_deg=-20.0,
    )

    assert command.tilt_about_x_deg == controller.max_tilt_deg
    assert command.tilt_about_y_deg == -controller.max_tilt_deg
    assert controller.servo_config.min_deg <= command.servox_deg
    assert command.servox_deg <= controller.servo_config.max_deg
    assert controller.servo_config.min_deg <= command.servoy_deg
    assert command.servoy_deg <= controller.servo_config.max_deg
