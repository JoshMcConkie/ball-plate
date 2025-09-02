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

def map_table_to_pixel(H, x, y):
    H_inv = np.linalg.solve(H,np.eye(3))
    p = H_inv @ np.array([x, y, 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
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

# Q for kalman filter
# Edit: had to seperate the sigmas because x-axis was lagging
def make_Q(dt, sigma_ax=20, sigma_ay=10):
    Q1D_x = sigma_ax**2 * np.array([
        [dt**4/4, dt**3/2],
        [dt**3/2, dt**2]
    ])
    Q1D_y = sigma_ay**2 * np.array([
        [dt**4/4, dt**3/2],
        [dt**3/2, dt**2]
    ])
    return np.block([
        [Q1D_x, np.zeros((2,2))],
        [np.zeros((2,2)), Q1D_y]
    ])

def run_kalman_filter(X,X_1,P_1,R,Q, dt):

    

    '''Physical AR process: X_t = AX_t-1 + w_t
    Explanation: The actual current state of the ball is a sum of the previous state
    changed by a step in time; in this case, A effectively adds a to the position states
    a time step of each velocity state. We then add w to account for environment shocks
    to the system.
    '''
    # recall that X = [x,y,vx,vy]
    I2 = np.eye(2)
    I4 = np.eye(4)
    A = np.block([[I2, dt*I2], [np.zeros((2,2)), I2]])

    X_hat_init = A@X_1

    ''' Calculating P is '''
    
    P_pred = A @ P_1 @ A.T + Q
    # Prevent collapse (especially on x)
    POS_FLOOR = 2e-2   # try 0.02; adjust to taste (0.01–0.05)
    P_pred[0,0] = max(P_pred[0,0], POS_FLOOR)
    P_pred[1,1] = max(P_pred[1,1], POS_FLOOR)
    print("Ppred_pos=", np.diag(P_pred)[:2])

    '''Sensor MA process: Y_t = HX_t + v_t
    Explanation: The measured state read by the camera is a sum of the actual state
    X and some measurement noise. The H projects what aspects of the true state vector
    are actually seen (by the camera in this case). Y_t is not an estimate (with a hat)
    because it is observed data, not a hidden measurement. So in other words:
    CameraCoords = [crop to x/y coords][full state vector] + [measurement noise]
    Here, we are using x/y coords instead of u/v pixel coords because this filter is
    called after the change of variables using our homogeneous mapping. 

    We rearrange the equation to solve for the residual, r
    '''
    # R = 0.5*(R + R.T)
    # w,V = np.linalg.eigh(R)
    # R = (V * np.clip(w, 1e-4, None)) @ V.T
    H_kf = np.array([[1,0,0,0],[0,1,0,0]])
    Y_hat = H_kf @ X_hat_init
    Y = H_kf @ X
    residual = Y - Y_hat
    S = H_kf @ P_pred @ H_kf.T + R
    # nis = residual.T @ np.linalg.solve(S, residual)  # scalar
    # if nis > 16.3:           # ~99% for 2D
    #     return X_hat_init, P_pred  # skip update this frame

    # print("diag(R)=", np.diag(R),
    #   " diag(S)=", np.diag(S))

    K = P_pred @ H_kf.T @ np.linalg.solve(S, I2)
    
    


    X_new = X_hat_init + K @ residual
    # P_new = (np.eye(4) - K @ H_kf) @ P_pred
    #Joseph form
    M = (I4 - K @ H_kf)
    P_new = M @ P_pred @ M.T + K @ R @ K.T
    P_new = 0.5 * (P_new + P_new.T)

    # after computing residual, S, K:
    nis = float(residual.T @ np.linalg.solve(S, residual))
    
    GATE_95 = 5.991
    # if nis > GATE_95:
    #     # Skip update on this frame: keep prediction
    #     x_new, P_new = X_hat_init, P_pred
    #     # (Optional) slightly inflate P to reflect uncertainty after a missed update
    #     P_new *= 1.02
    #     # (Optional) log: print(f"Gate hit: NIS={nis:.2f}")
    #     return x_new, P_new

    Kx, Ky = K[0,0], K[1,1]          # position gains
    print(f"NIS={nis:.2f}  Kx={Kx:.3f} Ky={Ky:.3f}")
    return X_new, P_new

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
# Freeze H
H_fixed = None

prev_t = None
X_filtered = None
X_filtered_1 = None

# Initializing measurement noise covariance for kalman filter (R)
initialized = False
Y_init_hist = []
R = None
P = np.diag([10.0,10.0,1.0,1.0])
# Start the webcam
feed = cv2.VideoCapture(CAMERA_ID)

# prevent dt bomb
MAX_DT = 0.1

print("Type (y) when ball is stationary on platform. Press (n) to skip calibration. Press esc to quit.\n")
while True:
    ret, frame = feed.read()
    cv2.imshow("Webcam Feed", frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('y'):
        break
    if k == 27:  # Esc
        feed.release()
        cv2.destroyAllWindows()
        raise SystemExit
print("Initializing. Keep ball still.")
    

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
    H_use = H_fixed if (initialized and H_fixed is not None) else H
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
                mapped = map_pixel_to_table(H_use, ball_cu,ball_cv)
                if mapped is None:
                    prev_t = now
                    continue
                cx,cy = mapped

            if not initialized and H_use is not None:
                    print("...")
                    if np.isfinite(cx) and np.isfinite(cy):
                        Y_init_hist.append([cx,cy])
                        print(f"Clean Entry {len(Y_init_hist)}: {cx}, {cy}")
                    if len(Y_init_hist) > 100:
                        
                        R = np.cov(np.array(Y_init_hist), rowvar=False)
                        R_use = R * .25
                        H_fixed = None if H is None else H.copy()
                        print(f"R-matrix: {R}")
                        initialized = True
            elif H_use is not None and initialized and cx_1 is not None and cy_1 is not None and prev_t is not None: 
                dt = now - prev_t
                
                if dt > MAX_DT:
                    print(f"Large dt detected ({dt:.2f}s), resetting filter.")
                    X_filtered_1 = None  # Reset the filter's memory
                    prev_t = now         # IMPORTANT: Update time to prevent repeated resets
                    continue             # Skip to the next frame
                # vx = (cx - cx_1)/dt
                # vy = (cy - cy_1)/dt
                Z = np.array([cx,cy,0,0])
                # X = np.array([cx,cy,vx,vy])
                if X_filtered_1 is None:
                    X_filtered_1 = np.array([cx,cy,0,0])
                Q = make_Q(dt)
                X_filtered, P = run_kalman_filter(Z,X_filtered_1,P,R_use,Q,dt)
                cx,cy,vx,vy = X_filtered
                
                kalman_u,kalman_v = map_table_to_pixel(H_use,cx,cy)
                cv2.circle(frame, (int(kalman_u), int(kalman_v)), int(30)+4, (255, 255, 255), 2)
                # print(f"Ball position: ({cx}, {cy}); Velocity (p/s): ({vx}, {vy})")

                # Only send packet if it follows the Hz constraint (50 Hz)
                # Packet Format: "float(x),float(y),int(vx),int(vy)"
                if SERIAL_ON and (now - last_send) >= SEND_INTERVAL:
                    # build a packet
                    packet = f"{int(cx)},{int(cy)},{int(vx)},{int(vy)}\n" #
                    try: # send/recieve from arduino through serial
                        ser.write(packet.encode('utf-8'))
                        print(packet)
                        echo = ser.readline().decode(errors='ignore').strip()
                        print("Echo:", echo)
                        last_send = now
                    except Exception as e:
                        print("Serial write failed:", e)
                
              
            elif initialized: # first frame rule
                print(f"Ball position: ({cx}, {cy}); Velocity (p/s): (N/A)")
                
            # Illustrate circle on out frame
            cv2.circle(frame, (int(u), int(v)), int(40), (0, 255, 255),2)
            cv2.circle(frame, (ball_cu, ball_cv), 4, (0, 0, 255), -1)                

            cx_1, cy_1, vx_1, vy_1 = cx, cy, vx, vy
            if X_filtered is not None:
                X_filtered_1 = X_filtered
            prev_t = now

    # cv2.imshow("Webcam Feed", mask)
    cv2.imshow("Webcam Feed", frame)
    if cv2.waitKey(1) == 27: # Esc key exit
        break

feed.release()
cv2.destroyAllWindows()