import subprocess
import time

import cv2
from cv2 import COLOR_BGR2GRAY, GaussianBlur, cvtColor
from cv2.typing import MatLike

from ball_plate.config import CAM_ID
from ball_plate.state import BallMeasurement

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

def get_mask(init_frame: MatLike, new_frame: MatLike, 
             diff_threshold=25)-> MatLike:
    filt_new_frame = filter_frame(new_frame)
    diff =  cv2.absdiff(filt_new_frame,init_frame)
    _, mask = cv2.threshold(diff,diff_threshold,255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def get_pos(mask: MatLike)->tuple[int,int]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x = 0
    y = 0

    return x,y

def get_ball_measurement(frame: MatLike)->BallMeasurement:
    # TODO: get ball location from cv2 feed via color value contour
    now = time.time()
    mask = get_mask(filt_init_frame, frame, diff_threshold=50)
    x,y = get_pos(mask)
    x_px, y_px =
    x_m, y_m =
    radius_px = 
    found = 
    confidence = 
    
    return BallMeasurement(now,x_px,y_px,x_m,y_m,
                           radius_px,found,confidence)