import subprocess
import time
import numpy as np
import cv2
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, GaussianBlur, cvtColor
from cv2.typing import MatLike

from ball_plate.config import CAM_ID, COLOR_BALL, TABLE_W_M, TABLE_H_M
from ball_plate.state import BallMeasurement

# Calibration state, set at startup via set_calibration()
PX_TO_M_X = None
PX_TO_M_Y = None
ORIGIN_PX = (0, 0)

def set_calibration(corners_px: list[tuple[int, int]]):
    '''Compute px->m scales and table-center origin from the 4 clicked table corners.
    Corners may be clicked in any order.'''
    global PX_TO_M_X, PX_TO_M_Y, ORIGIN_PX

    pts = np.array(corners_px, dtype=np.float64)
    # Order corners TL, TR, BR, BL: TL has min(x+y), BR has max(x+y),
    # TR has min(y-x), BL has max(y-x)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()  # y - x
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]

    width_px = (np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0
    height_px = (np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0

    PX_TO_M_X = TABLE_W_M / width_px
    PX_TO_M_Y = TABLE_H_M / height_px
    cx, cy = pts.mean(axis=0)
    ORIGIN_PX = (int(cx), int(cy))

def init_camera():
    cam = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cam

# TODO: Change mask to color filtered for colored ball.
def filter_frame(frame: MatLike)-> MatLike:
    gray = cvtColor(frame, COLOR_BGR2GRAY)
    return GaussianBlur(gray, (5,5),sigmaX=0)

def set_init_frame(frame: MatLike):
    return filter_frame(frame)

def get_mask_grayscale(init_frame: MatLike, new_frame: MatLike, 
             diff_threshold=25)-> MatLike:
    filt_new_frame = filter_frame(new_frame)
    diff =  cv2.absdiff(filt_new_frame,init_frame)
    _, mask = cv2.threshold(diff,diff_threshold,255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def get_mask_color(color: tuple[int,int,int], frame: MatLike)-> MatLike:
    hsv = cvtColor(frame, COLOR_BGR2HSV)
    lower = np.array([color[0] - 10, color[1] - 100, color[2] - 100])
    upper = np.array([color[0] + 10, color[1] + 100, color[2] + 100])
    mask = cv2.inRange(hsv, lower, upper) # binary mask of color
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def get_ball_px_coords(mask: MatLike)->tuple[int,int,int]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        contour = contours[0]
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            radius = int(M['m00'])
            return cx,cy,radius
    return None,None,None

def get_ball_table_coords(x_px: int, y_px: int)->tuple[float,float]:
    x = (x_px - ORIGIN_PX[0]) * PX_TO_M_X
    y = (y_px - ORIGIN_PX[1]) * PX_TO_M_Y
    return x,y

def get_ball_measurement(frame: MatLike)->BallMeasurement:
    # TODO: get ball location from cv2 feed via color value contour
    now = time.time()
    mask = get_mask_color(COLOR_BALL, frame)
    x_px, y_px, radius_px = get_ball_px_coords(mask)
    x_m, y_m = get_ball_table_coords(x_px, y_px)
    found = x_px is not None and y_px is not None
    confidence = radius_px / 100.0 if found else 0.0
    if not found:
        return BallMeasurement(now,0,0,0,0,0,False,0.0) # return dummy measurement
    return BallMeasurement(now,x_px,y_px,x_m,y_m,radius_px,True,confidence) # return actual measurement