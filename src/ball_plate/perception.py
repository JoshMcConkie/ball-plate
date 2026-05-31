import subprocess

import cv2
from cv2 import COLOR_BGR2GRAY, GaussianBlur, cvtColor
from cv2.typing import MatLike


CAM_ID = 0

def init_camera():
    cam = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cam


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