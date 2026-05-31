import time
import serial
import cv2
from math import atan2, sqrt, pi

from ball_plate import perception, planning, control, cam_tools

import subprocess

# Linux wait key codes
UP    = 65362
DOWN  = 65364
LEFT  = 65361
RIGHT = 65363

# Update Control Frequency
HERTZ = 50.0

SERIAL_ON = True

if SERIAL_ON:
    '''Time constraints'''
    last_send = 0.0 # Initialize
    SEND_INTERVAL = 1.0/HERTZ # aka 50 Hz


    '''Serial'''
    ser = serial.Serial('COM3', 115200, timeout=1)
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

# ---Main loop---
filt_init_frame = perception.filter_frame(frame0)
while True:
    # ==Perception==
    # Read IMU
    
    # Process Camera Feed

    # State Estimate

    # ==Planning==

    #==Control==
    # Send command to ESP32

    # Log
    
    ret, frame = feed.read()
    if not ret:
        break
    
    
    mask = perception.get_mask(filt_init_frame, frame, diff_threshold=50)
    x,y = perception.get_pos(mask)
    roll, pitch = 

    
    servox_deg, servoy_deg = 0,0
    # pos_to_angles(cmd_pos)

    
    recieve = control.send_recieve(servox_deg,servoy_deg, ser)
    if recieve is not None and len(recieve) == 6:
        ax,ay,az,gx,gy,gx = map(float,recieve)
        roll_deg_acc  = atan2(ay, az) * 180.0 / pi
        pitch_deg_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / pi


    cv2.imshow("Webcam Feed", mask)
    if cv2.waitKey(1) == 27: # Esc key exit
        break
feed.release()
cv2.destroyAllWindows()