import time
import serial
import cv2
from math import atan2, sqrt, pi

from ball_plate import perception, planning, control, cam_tools, serial_io
from ball_plate import estimator
from ball_plate.config import BAUD_RATE, CAMERA_HZ, CONTROL_HZ, IMU_HZ, REFERENCE_STATE, SERIAL_PORT, STATE_EST_HZ

import subprocess
from ball_plate.state import IMUReading, SystemState, TableState, BallMeasurement, BallState

# Linux wait key codes
UP    = 65362
DOWN  = 65364
LEFT  = 65361
RIGHT = 65363

SERIAL_ON = True

if SERIAL_ON:
    '''Serial'''
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=.001)
    time.sleep(2)
    banner = ser.readline().decode(errors='ignore').strip()
    print("Banner:", banner)

feed = perception.init_camera()

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

table_state = TableState(time.time(),0,0,0,0)
imu_data = None
while imu_data is None:
    imu_data = serial_io.fetch_packet(ser)


# ---Main loop---
filt_init_frame = perception.filter_frame(frame0)
ball_meas = perception.get_ball_measurement(filt_init_frame)
ball_state = estimator.get_ball_state()
system_state = control.get_system_state(ball_state,table_state,REFERENCE_STATE)
control_cmd = control.get_command(system_state, REFERENCE_STATE)

while True:
    # ==Perception==
    # Read IMU
    if (time.time() - imu_data.timestamp) > 1/IMU_HZ:
        imu_data = serial_io.fetch_packet(ser)
        if imu_data is None:
            continue
        table_state = estimator.get_table_state(table_state, imu_data)

    # Process Camera Feed
    if (time.time() - ball_meas.timestamp) > 1/CAMERA_HZ:
        ret, frame = feed.read()
        if not ret:
            break
        ball_meas = perception.get_ball_measurement(frame)

    # ==State Estimate==
    if (time.time() - table_state.timestamp) > 1/STATE_EST_HZ:
        ball_state = estimator.get_ball_state(ball_state, ball_meas)

    # ==Control==
    if (time.time() - imu_data.timestamp) > 1/CONTROL_HZ:
        system_state = control.get_system_state(ball_state,table_state,REFERENCE_STATE)
        control_cmd = control.get_command(system_state, REFERENCE_STATE)
        
        serial_io.send_packet(control_cmd, ser) # Send command to ESP32

    # Log    
    mask = perception.get_mask(filt_init_frame, frame, diff_threshold=50)
    x,y = perception.get_pos(mask)
    roll, pitch = 

    servox_deg, servoy_deg = 0,0
    # pos_to_angles(cmd_pos)

    cv2.imshow("Webcam Feed", mask)
    if cv2.waitKey(1) == 27: # Esc key exit
        break

feed.release()
cv2.destroyAllWindows()