
from dataclasses import dataclass
import subprocess
from time import time

import cv2
import numpy as np

from ball_plate.config import CameraConfig, PlateConfig

@dataclass(frozen=True)
class CoordinateMap:
    px_to_m_x: float
    px_to_m_y: float
    origin_px_x: int
    origin_px_y: int

    @classmethod
    def from_corners(cls, corner_pts:list[tuple[int, int]], plate_config: PlateConfig)->CoordinateMap:
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
        return CoordinateMap(plate_config.width_m / width_px,
                             plate_config.height_m / height_px,
                             cx,
                             cy)

class Camera:
    '''
    Camera class with methods specific to Linux systems.
    Run .calibrate() to setup camera.
    '''
    # Linux wait key codes
    UP    = 65362
    DOWN  = 65364
    LEFT  = 65361
    RIGHT = 65363

    def __init__(self, camera_config: CameraConfig):
        self.device_id: int = camera_config.device_id
        self.target_ball_color_hsv: list = camera_config.target_ball_color_hsv
        self.feed = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        self.px_to_meter_map: CoordinateMap | None = None


    def set_auto_exp(self,setting:bool):
        mode = '3' if setting else '1'
        subprocess.run(["v4l2-ctl", "-d", 
                    "/dev/video0", 
                    f"--set-ctrl=auto_exposure={mode}"])

    def set_exposure(self,level: int):
        subprocess.run(["v4l2-ctl", "-d", 
                    "/dev/video0", 
                    f"--set-ctrl=brightness={level}"],
                    check=True)
        
    def get_exposure(self):
        result =  subprocess.run(
                ["v4l2-ctl", "-d", "/dev/video0", "--get-ctrl=brightness"],
                capture_output=True,
                text=True,
                check=True,
            )
        level = int(result.stdout.split(":")[1].strip())
        return level

    def exposure_warmup(self):
        self.set_auto_exp(True)
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            ret, frame = self.feed.read()
            if not ret:
                continue

            cv2.imshow("Auto exposure warmup", frame)
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        self.set_auto_exp(False)

    def adjust_exposure_manual(self):
        exposure = self.get_exposure()
        # Window to adjust exposure
        while True:
            ret0, frame0 = self.feed.read()
            if not ret0:
                break
            cv2.imshow("Init Feed", frame0)
            key = cv2.waitKeyEx(1)
            if key == ord(' '): # Esc key exit
                break
            elif key == self.UP:
                exposure += 5
                self.set_exposure(exposure)
                print(self.get_exposure())
            elif key == self.DOWN:
                exposure -= 5
                self.set_exposure(exposure)
                print(self.get_exposure())

        cv2.destroyAllWindows()

    
    def select_plate_corners(self)->list[tuple[int,int]]:
        #====Calibration camera px to plate meters coordinate map=====
        corner_pts = []

        def on_corner_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(corner_pts) < 4:
                corner_pts.append((x, y))

        cv2.namedWindow("Select Plate Corners")
        cv2.setMouseCallback("Select Plate Corners", on_corner_click)

        while True:
            ret, frame = self.feed.read()
            if not ret:
                break

            display = frame.copy()
            cv2.putText(display, 
                        "Click 4 table corners | r: reset | space: confirm",
                        (10, 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.6, 
                        (0, 255, 0), 
                        2)
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
        return corner_pts
    

    def calibrate(self, plate_config):
        self.exposure_warmup()
        self.adjust_exposure_manual()
        corner_pts = self.select_plate_corners()
        self.px_to_meter_map = CoordinateMap.from_corners(corner_pts, plate_config)
        


