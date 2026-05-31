import time

import cv2

from ball_plate import perception, cam_tools

import subprocess

# Linux wait key codes
UP    = 65362
DOWN  = 65364
LEFT  = 65361
RIGHT = 65363

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
    ret, frame = feed.read()
    if not ret:
        break
    
    # ==Perception==
    mask = perception.get_mask(filt_init_frame, frame, diff_threshold=50)
    x,y = perception.get_pos(mask)
    roll, pitch = 

    # ==Planning==

    # pos_to_angles(cmd_pos)

    #==Control==
    


    cv2.imshow("Webcam Feed", mask)
    if cv2.waitKey(1) == 27: # Esc key exit
        break
feed.release()
cv2.destroyAllWindows()