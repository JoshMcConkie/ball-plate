import cv2
import numpy as np
from pupil_apriltags import Detector

# ===================== USER SETUP =====================
# Choose your table frame. Example: origin at center, mm units.
# Measure/tag the CENTER of each AprilTag on the table and put its (x, y) here.
# Use at least 4 tags; 6–8 gives redundancy.
TAG_TABLE_COORDS = {
    0: (-90.0, -90.0),
    1: (-90.0, 90.0),
    2: (90.0, 90.0),
    3: (90.0, -90.0)
}

CAM_INDEX = 1  # your webcam index
UNDISTORT = False  # set True if you have camera intrinsics below
# Fill these only if UNDISTORT=True (use a real calibration for best accuracy)
K = np.eye(3, dtype=np.float32)            # camera matrix
D = np.zeros((1,5), dtype=np.float32)      # distortion coeffs (k1,k2,p1,p2,k3)

# ===================== APRILTAG DETECTOR =====================
detector = Detector(
    families='tag36h11',
    nthreads=4,
    quad_decimate=2.0,     # speed/accuracy tradeoff: lower = sharper, slower
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25
)

# ===================== HELPERS =====================
def solve_homography(uv_pts, xy_pts, method=cv2.RANSAC):
    """
    uv_pts: Nx2 image pixels (float32)
    xy_pts: Nx2 table coords in mm (float32)
    returns H (3x3) or None
    """
    if len(uv_pts) < 4:
        return None
    # If you have >=5 points, RANSAC helps reject bad detections.
    # With exactly 4 points, RANSAC falls back to a direct solve.
    H, mask = cv2.findHomography(uv_pts, xy_pts, method=method, ransacReprojThreshold=2.0)
    return H

def map_pixel_to_table(H, u, v):
    """Homogeneous mapping: [X',Y',W']^T = H [u,v,1]^T, then (x,y) = (X'/W', Y'/W')."""
    p = H @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(p[2]) < 1e-9:
        return None
    return (p[0] / p[2], p[1] / p[2])

def draw_axes(img, H, scale=50):
    """Draw table frame axes mapped into the image (purely for visualization sanity)."""
    # table origin and unit axes in table coords
    origins = np.array([[0,0],[scale,0],[0,scale]], dtype=np.float64)
    # invert H to go table->image for drawing
    try:
        Hinv = np.linalg.inv(H)
    except np.linalg.LinAlgError:
        return
    pix = []
    for (x,y) in origins:
        q = Hinv @ np.array([x, y, 1.0])
        q = q[:2] / q[2]
        pix.append(tuple(q.astype(int)))
    # draw: origin = white dot, +x = line (origin->red), +y = line (origin->green)
    cv2.circle(img, pix[0], 5, (255,255,255), -1)
    cv2.line(img, pix[0], pix[1], (0,0,255), 2)
    cv2.line(img, pix[0], pix[2], (0,255,0), 2)
    cv2.putText(img, "+x", pix[1], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
    cv2.putText(img, "+y", pix[2], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

def order_pairs_by_id(detections, id_to_xy):
    """Return matched pixel & table coord arrays (uv, xy) for tags whose IDs are in id_to_xy."""
    uv, xy = [], []
    for d in detections:
        tid = d.tag_id
        if tid in id_to_xy:
            u, v = d.center  # (col=x, row=y) in pixels
            uv.append([u, v])
            xy.append(id_to_xy[tid])
    return np.array(uv, dtype=np.float32), np.array(xy, dtype=np.float32)

# ===================== MAIN LOOP =====================
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)

# Try to reduce auto-flux (not guaranteed; depends on your cam/driver)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 or 0.25 depending on backend
# cap.set(cv2.CAP_PROP_EXPOSURE, -6)    # tweak as needed

last_H = None
hold_frames = 0
HOLD_FRAMES_MAX = 10  # how long to keep using last_H if detections drop below 4

while True:
    ok, frame = cap.read()
    if not ok:
        break

    if UNDISTORT:
        frame = cv2.undistort(frame, K, D)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    # collect matched points by known IDs
    uv, xy = order_pairs_by_id(detections, TAG_TABLE_COORDS)

    # solve/update H if we have enough tags
    if len(uv) >= 4:
        H = solve_homography(uv, xy, method=cv2.RANSAC if len(uv) >= 5 else 0)
        if H is not None:
            last_H = H
            hold_frames = 0
    else:
        H = last_H
        hold_frames += 1

    # Visualize detections
    for d in detections:
        c = tuple(np.round(d.center).astype(int))
        cv2.circle(frame, c, 6, (255, 0, 255), 2)
        cv2.putText(frame, f"id{d.tag_id}", (c[0]+6, c[1]-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1, cv2.LINE_AA)
        # draw corners
        for pt in d.corners.astype(int):
            cv2.circle(frame, tuple(pt), 3, (0,255,255), -1)

    # If we have a homography, draw axes and demo a mapping
    if H is not None:
        draw_axes(frame, H, scale=50)

        # Example: map the image center to table coords
        h, w = frame.shape[:2]
        table_xy = map_pixel_to_table(H, w/2, h/2)
        if table_xy is not None:
            tx, ty = table_xy
            cv2.putText(frame, f"Center->table: ({tx:.1f}, {ty:.1f}) mm",
                        (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)

        # Warn if we’re running on a stale H too long
        if len(uv) < 4 and hold_frames > HOLD_FRAMES_MAX:
            cv2.putText(frame, "HOLDING LAST H (low tags)", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,140,255), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, "NO HOMOGRAPHY (need >=4 known tags)",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2, cv2.LINE_AA)

    cv2.imshow("AprilTags → Homography → Table coords", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
