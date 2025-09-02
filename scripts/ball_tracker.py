import cv2
import numpy as np
import time
import serial
from pupil_apriltags import Detector

SERIAL_ON = True
CAMERA_ID = 0

# Table
TAG_TABLE_COORDS = {
    0: (-90.0, -90.0),
    1: (-90.0, 90.0),
    2: (90.0, 90.0),
    3: (90.0, -90.0)
}
UNDISTORT = False


# -------- APRILTAG DETECTOR ------
detector = Detector(
    families='tag36h11',
    nthreads=4,
    quad_decimate=2.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25
)

# ---------- HELPERS --------
def homography(uv_pts, x_pts):

    if len(uv_pts) < 4:
        return None
    H, mask = cv2.findHomography(uv_pts, x_pts, cv2.RANSAC, ransacReprojThreshold=2.0)
    return H
def map_pixel_to_table(H, u, v):
    """Homogeneous mapping"""
    p = H @ np.array([u,v,1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9: # avoid noise
        return None
    return (p[0] / p[2], p[1] / p[2])

def order_pairs_by_id(detections, id_to_xy):
    uv, xy = [],[]
    for d in detections:
        tid = d.tag_id
        if tid in id_to_xy:
            u,v = d.center
            uv.append([u,v])
            xy.append(id_to_xy[tid])
    return np.array(uv, dtype=np.float32), np.array(xy, dtype=np.float32)



if SERIAL_ON:
    '''Time constraints'''
    last_send = 0.0 # Initialize
    SEND_INTERVAL = 1.0/50.0 # aka 50 Hz


    '''Serial'''
    ser = serial.Serial('COM3', 115200, timeout=1)
    time.sleep(2)
    banner = ser.readline().decode(errors='ignore').strip()
    print("Banner:", banner)

'''Tracking + Packet send'''
cx_1 = None # previous centroid x coordinate
cy_1 = None # previous centroid y coordinate
vx = 0
vy = 0
vx_1 = 0
vy_1 = 0
prev_t = None

# Start the webcam
feed = cv2.VideoCapture(CAMERA_ID)
ret0,frame0 = feed.read()

while True:
    # Read frame
    ret, frame = feed.read()
    if not ret:
        break

    # table
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
    tags_uv, tags_xy = order_pairs_by_id(detections, TAG_TABLE_COORDS)
    
    H = homography(tags_uv,tags_xy)
    # ball
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 

    # The ball in use is green. Change the below bounds for different color.
    lower_ball_color = np.array([40,70,70]) # green
    upper_ball_color = np.array([80, 255, 255]) # green

    # lower_ball_color = np.array([95, 50, 40])   # H:100, medium S, low V
    # upper_ball_color = np.array([115, 150, 120]) # H:130, full S/V


    # Binary mask (green:1 else:0)
    mask = cv2.inRange(hsv, lower_ball_color, upper_ball_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask,None, iterations=2)
    # Identify the ball contour within the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if contours: # Prevents break if no green in frame

        c = max(contours, key=cv2.contourArea)

        # Initial circle around ball pixels
        (u,v), radius = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

    
        # getting center point of contour
        if M["m00"] > 0 and radius > 8:
            now = time.time()
            ball_cu = int(M["m10"] /M["m00"])
            ball_cv = int(M["m01"] /M["m00"])
            if H is None:
                cx,cy = cx_1,cy_1
            else:
                cx, cy = map_pixel_to_table(H, ball_cu,ball_cv)
            if cx_1 is not None and cy_1 is not None and prev_t is not None: 
                dt = now - prev_t
                vx = (cx - cx_1)/dt
                vy = (cy - cy_1)/dt
                alpha = .8
                fvx = alpha*vx+(1-alpha)*vx_1
                fvy = alpha*vy+(1-alpha)*vy_1
                # print(f"Ball position: ({cx}, {cy}); Velocity (p/s): ({vx}, {vy})")

                # Only send packet if it follows the Hz constraint (50 Hz)
                # Packet Format: "float(x),float(y),int(vx),int(vy)"
                if SERIAL_ON and (now - last_send) >= SEND_INTERVAL:
                    # build a packet
                    packet = f"{int(cx)},{int(cy)},{int(fvx)},{int(fvy)}\n" #
                    try: # send/recieve from arduino through serial
                        ser.write(packet.encode('utf-8'))
                        print(packet)
                        echo = ser.readline().decode(errors='ignore').strip()
                        print("Echo:", echo)
                        last_send = now
                    except Exception as e:
                        print("Serial write failed:", e)
              
            else: # first frame rule
                print(f"Ball position: ({cx}, {cy}); Velocity (p/s): (N/A)")
                
            # Illustrate circle on out frame
            cv2.circle(frame, (int(u), int(v)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (ball_cu, ball_cv), 4, (0, 0, 255), -1)
            vx_1,vy_1 = vx, vy
            cx_1, cy_1 = cx, cy
            prev_t = now

    # cv2.imshow("Webcam Feed", mask)
    cv2.imshow("Webcam Feed", frame)
    if cv2.waitKey(1) == 27: # Esc key exit
        break

feed.release()
cv2.destroyAllWindows()