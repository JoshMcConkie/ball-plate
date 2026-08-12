from dataclasses import dataclass
import time
import numpy as np
import cv2
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, GaussianBlur, VideoCapture, cvtColor
from cv2.typing import MatLike

from ball_plate.config import CAM_ID, BALL_COLOR, TABLE_W_M, TABLE_H_M
from ball_plate.state import BallMeasurement

@dataclass(frozen=True)
class CoordinateMap:
    px_to_m_x: float
    px_to_m_y: float
    origin_px_x: int
    origin_px_y: int

    @staticmethod
    def from_corners(corner_pts:list[tuple[int, int]])->CoordinateMap:
        '''Compute px->m scales and table-center origin from the 4 clicked table corners.
        Corners may be clicked in any order.'''

        pts = np.array(corner_pts, dtype=np.float32)
        # Order corners TL, TR, BR, BL: TL has min(x+y), BR has max(x+y),
        # TR has min(y-x), BL has max(y-x)
        s = pts.sum(axis=1)
        d = np.diff(pts, axis=1).ravel()  # y - x
        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]
        tr = pts[np.argmin(d)]
        bl = pts[np.argmax(d)]

        width_px = float((np.linalg.norm(tr - tl) + np.linalg.norm(br - bl)) / 2.0)
        height_px = float((np.linalg.norm(bl - tl) + np.linalg.norm(br - tr)) / 2.0)

        cx, cy = pts.mean(axis=0)   
        return CoordinateMap(TABLE_W_M / width_px,
                             TABLE_H_M / height_px,
                             cx,
                             cy)
    

def init_camera():
    cam = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cam

class BallDetector:
    def __init__(self,
                 coord_map: CoordinateMap,
                 color: tuple[int,int,int]
                 ):
        self.map = coord_map
        self.color = color
        self.last_meas = BallMeasurement(0,0,0,0,0,0,False) # initial dummy measurement

    def _build_contour_mask(self, frame: MatLike)-> MatLike:
        # Build mask
        hsv = cvtColor(frame, COLOR_BGR2HSV)
        lower = np.array([self.color[0] - 10, self.color[1] - 100, self.color[2] - 100])
        upper = np.array([self.color[0] + 10, self.color[1] + 100, self.color[2] + 100])
        mask = cv2.inRange(hsv, lower, upper) # binary mask of color
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _meas_px(self,mask)->tuple[int,int,int] | None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour = contours[0]
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                radius = int(M['m00'])
                return cx,cy,radius
        return None

    def px_to_meter(self,x_px: int, y_px: int)->tuple[float,float]:
        # Image rows increase downward, so negate to make +y point up.
        x = (x_px - self.map.origin_px_x) * self.map.px_to_m_x
        y = (self.map.origin_px_y - y_px) * self.map.px_to_m_y
        return x,y

    def measure(self,frame: MatLike)->BallMeasurement:
        now = time.monotonic()
        mask = self._build_contour_mask(frame)
        meas_px = self._meas_px(mask)
        if meas_px is None:
            return self.last_meas # return previous measurement
        x_px, y_px, radius_px = meas_px
        x_m, y_m = self.px_to_meter(x_px, y_px)
        self.last_meas = BallMeasurement(now,x_px,y_px,x_m,y_m,radius_px,True)
        return self.last_meas # return actual measurement

    def measure_px_only(self,frame:MatLike)->tuple[float,float] | None:
        mask = self._build_contour_mask(frame)
        meas_px = self._meas_px(mask)
        if meas_px is None:
            return None
        x_px, y_px, _ = meas_px
        return x_px, y_px