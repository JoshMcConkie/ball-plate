import time
import serial
import cv2
import numpy as np

from ball_plate import control, cam_tools, serial_io
from ball_plate.estimation.calibration.tools import calibrate_ball
from ball_plate.estimation.table_estimator import PlateEstimator
from ball_plate.estimation.ball_estimator import BallEstimator
from ball_plate.config import BALL_COLOR, BAUD_RATE, CAMERA_HZ, CONTROL_HZ, DEBUG_HZ, IMU_HZ, REFERENCE_STATE, SERIAL_PORT

from ball_plate.estimation.models import BallOnPlateModel, PlateModel
from ball_plate.perception import ball, imu
from ball_plate.state import PlateState, BallState

# Linux wait key codes
UP    = 65362
DOWN  = 65364
LEFT  = 65361
RIGHT = 65363

SERIAL_ON = True

if SERIAL_ON:
    '''Serial'''
    ser = serial_io.open_serial(SERIAL_PORT, BAUD_RATE, timeout=.001)
    print("Serial connected on:", ser.port)
    time.sleep(2)
    banner = ser.readline().decode(errors='ignore').strip()
    print("Banner:", banner)

feed = ball.init_camera()

# ---Linux specific exposure solution---

cam_tools.set_auto_exp(True)

# feed.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
# feed.set(cv2.CAP_PROP_EXPOSURE, 100)
# feed.set(cv2.CAP_PROP_GAIN, 0)

# ---Initialize Camera position and exposure---

start = time.monotonic()
while time.monotonic() - start < 2.0:
    ret, frame = feed.read()
    if not ret:
        continue

    cv2.imshow("Auto exposure warmup", frame)
    cv2.waitKey(1)
cv2.destroyAllWindows()
cam_tools.set_auto_exp(False)

exposure = cam_tools.get_exposure_linux()

# Window to adjust exposure
while True:
    ret0, frame0 = feed.read()
    if not ret0:
        break
    cv2.imshow("Init Feed", frame0)
    key = cv2.waitKeyEx(1)
    if key == ord(' '): # Esc key exit
        break
    elif key == UP:
        exposure += 5
        cam_tools.set_exposure_linux(exposure)
        print(cam_tools.get_exposure_linux())
    elif key == DOWN:
        exposure -= 5
        cam_tools.set_exposure_linux(exposure)
        print(cam_tools.get_exposure_linux())

cv2.destroyAllWindows()


#====Calibration camera px to plate meters coordinate map=====
corner_pts = []

def on_corner_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(corner_pts) < 4:
        corner_pts.append((x, y))

cv2.namedWindow("Select Plate Corners")
cv2.setMouseCallback("Select Plate Corners", on_corner_click)

while True:
    ret0, frame0 = feed.read()
    if not ret0:
        break

    display = frame0.copy()
    cv2.putText(display, "Click 4 table corners | r: reset | space: confirm",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    for pt in corner_pts:
        cv2.circle(display, pt, 5, (0, 0, 255), -1)
    if len(corner_pts) >= 2:
        cv2.polylines(display, [np.array(corner_pts)], len(corner_pts) == 4, (0, 255, 255), 2)
    cv2.imshow("Select Plate Corners", display)

    key = cv2.waitKeyEx(1)
    if key == ord('r'):
        corner_pts.clear()
    elif key == ord(' ') and len(corner_pts) == 4:
        break
cv2.destroyAllWindows()


#====Initialize perception objects====

coordinate_map = ball.CoordinateMap.from_corners(corner_pts)
ball_detector = ball.BallDetector(coordinate_map, BALL_COLOR)
imu_reader = imu.IMUReader(ser)

#====Calibrate objects for filtering====
print("Calibrating camera noise. Do not move anything.")
ball_cal = calibrate_ball(
    feed,
    ball_detector,
    estimate_process_noise=True
)

#====Initialize models objects====
ball_model = BallOnPlateModel(acc_var=ball_cal.acc_var)
plate_model = PlateModel()

#====Initialize estimation objects====
ball_estimator = BallEstimator(ball_model, meas_cov=ball_cal.meas_cov)
plate_estimator = PlateEstimator(plate_model)



#====Initialize measurement objects====
imu_meas = None
while imu_meas is None:
    imu_meas = imu_reader.measure()

ret, frame = feed.read()
if not ret:
    raise Exception("Failed to read initial frame")
ball_meas = ball_detector.measure(frame)

#====Initialize state/command objects====
ball_state = BallState(ball_meas.timestamp, ball_meas.x_m, ball_meas.y_m, 0.0, 0.0)
plate_state = PlateState(time.monotonic(),0,0,0,0)
system_state = control.get_system_state(ball_state,plate_state,REFERENCE_STATE)
control_cmd = control.get_command(system_state, REFERENCE_STATE)

#====Initialize clock variables====
log_timestamp = time.monotonic()
last_imu_poll = time.monotonic()
last_control = time.monotonic()



# ================Main loop=====================
while True:
    now = time.monotonic()

    # ==Perception==
    # Read IMU
    if now - last_imu_poll >= 1/IMU_HZ:
        last_imu_poll = now
        new_imu = imu_reader.measure()
        if new_imu is not None:
            imu_meas = new_imu
            plate_state = plate_estimator.estimate_vanilla_acc_only(plate_state, imu_meas)

    # Process Camera Feed
    if (time.monotonic() - ball_meas.timestamp) > 1/CAMERA_HZ:
        ret, frame = feed.read()
        if not ret:
            break
        ball_meas = ball_detector.measure(frame)

    # ==State Estimate==
    # Only update from a fresh, valid (ball found) measurement; otherwise hold
    # the last known ball state so a lost ball doesn't snap.
    if ball_meas.found and ball_meas.timestamp > ball_state.timestamp:
        ball_state = ball_estimator.estimate_vanilla(ball_state, ball_meas)

    # ==Control==
    if now - last_control >= 1/CONTROL_HZ:
        last_control = now
        system_state = control.get_system_state(ball_state,plate_state,REFERENCE_STATE)
        control_cmd = control.get_command(system_state, REFERENCE_STATE)
        
        serial_io.send_packet(control_cmd, ser) # Send command to ESP32

    # ==Logging==
    if (time.monotonic() - log_timestamp) > 1/DEBUG_HZ:
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
feed.release()
cv2.destroyAllWindows()
