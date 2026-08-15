import time

import cv2

from ball_plate import camera, control, serial_tools
from ball_plate.calibration import BallCalibrator, IMUCalibrator
from ball_plate.config import load_system_config
from ball_plate.estimation.ball_estimator import BallEstimator
from ball_plate.estimation.models import BallOnPlateModel, IMUFusionModel
from ball_plate.estimation.plate_estimator import PlateEstimator
from ball_plate.perception import ball, imu
from ball_plate.state import BallState, PlateState, ReferenceState

config = load_system_config()

controller = control.ServoController(controller_config=config.controller,
                             plate_config=config.plate, servo_config=config.servos)

serial_io = serial_tools.SerialIO(serial_config=config.serial)

camera = camera.Camera(camera_config=config.camera)
camera.calibrate(config.plate)

runtime_rates = config.runtime.rates_hz

reference_state = ReferenceState.from_config(config.reference)

#====Calibrate objects for perception and filtering====

imu_calibration = IMUCalibrator(controller=controller, serial_io=serial_io,
                               sample_freq=150, sample_count=300).calibrate()
imu_reader = imu.IMUReader(serial_io,imu_calibration)

ball_detector = ball.BallDetector(camera)
ball_calibration = BallCalibrator(camera=camera, ball_detector=ball_detector).calibrate()

#====Initialize models objects====
ball_model = BallOnPlateModel(acc_var=ball_calibration.acc_var)
plate_model = IMUFusionModel(imu_send_rate=200) #TODO: implement IMUConfig Class

#====Initialize estimation objects====
ball_estimator = BallEstimator(ball_model, meas_cov=ball_calibration.meas_cov)
plate_estimator = PlateEstimator(plate_model)



#====Initialize measurement objects====
imu_meas = None
while imu_meas is None:
    imu_meas = imu_reader.measure()

ret, frame = camera.feed.read()
if not ret:
    raise Exception("Failed to read initial frame")
ball_meas = ball_detector.measure(frame)

#====Initialize state/command objects====
ball_state = BallState(ball_meas.timestamp, ball_meas.x_m, ball_meas.y_m, 0.0, 0.0)
plate_state = PlateState(time.monotonic(),0,0)
system_state = controller.get_system_state(ball_state,plate_state,reference_state)
control_cmd = controller.get_command(system_state, reference_state)

#====Initialize clock variables====
log_timestamp = time.monotonic()
last_imu_poll = time.monotonic()
last_control = time.monotonic()



# ================Main loop=====================
while True:
    now = time.monotonic()

    # ==Perception==
    # Read IMU
    if now - last_imu_poll >= 1/runtime_rates.imu_read:
        last_imu_poll = now
        new_imu = imu_reader.measure()
        if new_imu is not None:
            imu_meas = new_imu
            plate_state = plate_estimator.estimate_vanilla_acc_only(plate_state, imu_meas)

    # Process Camera Feed
    if (time.monotonic() - ball_meas.timestamp) > 1/runtime_rates.camera_capture:
        ret, frame = camera.feed.read()
        if not ret:
            break
        ball_meas = ball_detector.measure(frame)

    # ==State Estimate==
    # Only update from a fresh, valid (ball found) measurement; otherwise hold
    # the last known ball state so a lost ball doesn't snap.
    if ball_meas.found and ball_meas.timestamp > ball_state.timestamp:
        ball_state = ball_estimator.estimate_vanilla(ball_state, ball_meas)

    # ==Control==
    if now - last_control >= 1/runtime_rates.control:
        last_control = now
        system_state = controller.get_system_state(ball_state,plate_state,reference_state)
        control_cmd = controller.get_command(system_state, reference_state)
        
        serial_io.send_packet(control_cmd) # Send command to ESP32

    # ==Logging==
    if (time.monotonic() - log_timestamp) > 1/runtime_rates.debug_output:
        log_timestamp = time.monotonic()
        # print(f"Timestamp: {log_timestamp}\n", 
        #     f"Ball State: {ball_state}\n",
        #     f"Plate State: {plate_state}\n", 
        #     f"Reference State: {REFERENCE_STATE}\n",
        #     f"Control Command: {control_cmd}\n")
        cv2.circle(frame, (ball_meas.x_px,ball_meas.y_px),
                   5, (0, 255, 0), 1)
        cv2.imshow("Ball Position", frame)
        print(f"Ball Position: {ball_state.x}, {ball_state.y} | "
              f"Servo Command: {control_cmd.servox_deg:.2f}, "
              f"{control_cmd.servoy_deg:.2f}")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.feed.release()
cv2.destroyAllWindows()
