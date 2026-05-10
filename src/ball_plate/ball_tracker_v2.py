import cv2

from ball_plate import perception
CAM_ID = 0

# ===Perception/Estimation====
def init_camera():
    cam = cv2.VideoCapture(CAM_ID)
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cam


def anchor_coordinates():
    pass

def get_ball_xy():
    pass



# ===Planning===
def PID_control():
    pass

# ===Control===

feed = init_camera()
ret0, frame0 = feed.read()
filt_init_frame = perception.filter_frame(frame0)
while True:
    ret, frame = feed.read()
    if not ret:
        break

    mask = perception.get_mask(filt_init_frame, frame)


    cv2.imshow("Webcam Feed", mask)
    if cv2.waitKey(1) == 27: # Esc key exit
        break
feed.release()
cv2.destroyAllWindows()