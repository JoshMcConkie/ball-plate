from math import asin, degrees, radians, sin

from ball_plate.config import ControllerConfig, PlateConfig, ServoConfig
from ball_plate.state import *


class ServoController:
    def __init__(self, controller_config: ControllerConfig,
                 plate_config: PlateConfig, servo_config: ServoConfig):
        self.kp = controller_config.kp
        self.ki = controller_config.ki
        self.kd = controller_config.kd
        self.max_tilt_deg = controller_config.max_tilt_deg
        self.alpha = controller_config.derivative_filter_alpha
        self.max_integral = self.max_tilt_deg / self.ki
        self.servo_config = servo_config
        self.plate = plate_config

    def reset(self)->None:
        self._previous_error_x: float | None = None
        self._previous_error_y: float | None = None
        self._previous_error_vx = 0.0
        self._previous_error_vy = 0.0
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._last_timestamp: float | None = None

    def get_servo_angles(self,tilt_about_x_deg: float, tilt_about_y_deg: float)->tuple[float,float]:
        # Edge lift needed for tilt = (table/2)*sin(tilt); servo arm rotates by asin(lift/arm)
        arg_x = (self.plate.width_m/(2*self.servo_config.arm_length_m)) * sin(radians(tilt_about_y_deg))
        arg_y = (self.plate.height_m/(2*self.servo_config.arm_length_m)) * sin(radians(tilt_about_x_deg))
        # Clamp to asin domain in case the requested tilt exceeds the arm's reach.
        # asin(...) is the servo deflection from flat; offset by the firmware
        # neutral (90 deg) so a balanced plate commands flat instead of an extreme.
        servox_deg = self.servo_config.center_deg + degrees(asin(max(-1.0, min(1.0, arg_x))))
        servoy_deg = self.servo_config.center_deg + degrees(asin(max(-1.0, min(1.0, arg_y))))
        return servox_deg, servoy_deg


    def get_command(self,system: SystemState, ref: ReferenceState)->ControlCommand:
        error_x = ref.x_goal - system.ball.x
        error_y = ref.y_goal - system.ball.y

        if self._last_timestamp is None:
            dt = 0.0
        else:
            dt = system.timestamp - self._last_timestamp
            dt = max(0.0, dt)
        self._last_timestamp = system.timestamp
        self._integral_x += error_x * dt
        self._integral_y += error_y * dt
        if self._integral_x > self.max_integral:
            self._integral_x = self.max_integral
        elif self._integral_x < -self.max_integral:
            self._integral_x = -self.max_integral
        if self._integral_y > self.max_integral:
            self._integral_y = self.max_integral
        elif self._integral_y < -self.max_integral:
            self._integral_y = -self.max_integral

        # A sensor state can be reused by more than one control iteration.
        if dt > 0.0 and self._prev_error_x is not None and self._prev_error_y is not None:
            error_vx = self.alpha*(error_x - self._prev_error_x) / dt + (1-self.alpha) * self._previous_error_vx
            error_vy = self.alpha*(error_y - self._prev_error_y) / dt + (1-self.alpha) * self._previous_error_vy
        else:
            error_vx = 0.0
            error_vy = 0.0

        if dt > 0.0 or self._prev_error_x is None:
            self._prev_error_x = error_x
            self._prev_error_y = error_y
        # Convert the position and velocity error into the plate tilt.
        tilt_about_y_deg = (self.kp * error_x + self.ki * self._integral_x + self.kd * error_vx)
        tilt_about_x_deg = (self.kp * error_y + self.ki * self._integral_y + self.kd * error_vy)

        if tilt_about_y_deg > self.max_tilt_deg:
            tilt_about_y_deg = self.max_tilt_deg
        elif tilt_about_y_deg < -self.max_tilt_deg:
            tilt_about_y_deg = -self.max_tilt_deg
        if tilt_about_x_deg > self.max_tilt_deg:
            tilt_about_x_deg = self.max_tilt_deg
        elif tilt_about_x_deg < -self.max_tilt_deg:
            tilt_about_x_deg = -self.max_tilt_deg

        servox_deg, servoy_deg = self.get_servo_angles(tilt_about_x_deg, tilt_about_y_deg)

        return ControlCommand(system.timestamp, tilt_about_x_deg, tilt_about_y_deg, servox_deg, servoy_deg)

    def get_system_state(self,ball_state: BallState, table_state: PlateState, ref_state: ReferenceState)->SystemState:
        timestamp = max(ball_state.timestamp,table_state.timestamp)
        return SystemState(timestamp, ball_state,table_state,ref_state)


    def get_max_tilt_cmds(self)->tuple[str, str]:

        max_servo_x, max_servo_y = self.get_servo_angles(self.max_tilt_deg, self.max_tilt_deg)
        return (f"{max_servo_x},{self.servo_config.center_deg}", f"{max_servo_y},{self.servo_config.center_deg}")

    def get_level_command(self):
        pass
