import numpy as np

'''
Inverse Kinematic approach. More appropriate, avoids three sphere
intersection.

Current config:
    * only configured for one pitch servo currently
    - roll/pitch -> servo angle

'''


RADIUS = 20     # Half width of table
P_T = np.array([[RADIUS]
                ,[0]
                ,[0]])

L1 = 2.5 # servo arm (servo to elbow)
L2 = 5.5 # tie rod (elbow to anchor)

roll = 5 * np.pi / 180 # rotation about x-axis
tilt = -5 * np.pi / 180 # rotation about y-axis

ORIGIN = np.zeros((3,1))

def Rx(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1,0,0],
                     [0,c,-s],
                     [0,s,c]], float)

def Ry(tilt):
    c, s = np.cos(tilt), np.sin(tilt)
    return np.array([[ c,0, s],
                     [ 0,1, 0],
                     [-s,0, c]], float)

P_GLOBAL = P_T
def table_to_global(P_table, roll, tilt, O=np.zeros((3,1))):
    R = Ry(tilt) @ Rx(roll)
    return O + R @ P_table

TILT_ANCHOR_TABLE_COORDS = np.array([[RADIUS]
                                  ,[0]
                                  ,[0]])
TILT_SERVO_GLOB_COORDS = np.array([[RADIUS]
                     ,[L1]
                     ,[-L2]])
TILT_SERVO_TO_GLOBAL = np.array([[0,0,1]
                     ,[1,0,0]
                     ,[0,1,0]])

ROLL_ANCHOR_TABLE_COORDS = np.array([[0]
                                  ,[RADIUS]
                                  ,[0]])

ROLL_SERVO_GLOB_COORDS = np.array([[L1]
                     ,[RADIUS]
                     ,[-L2]])
ROLL_SERVO_TO_GLOBAL = np.array([[-1,0,0]
                     ,[0,0,1]
                     ,[0,1,0]])

def global_to_servo(P_global, S_global, R_servo_to_global):
    R_glob_to_serv = R_servo_to_global.T
    return R_glob_to_serv @ (P_global - S_global)

'''
get_servo_angle: {Table point (T basis),roll,tilt} -> {Servo angle}
'''

def get_servo_angles(roll, tilt): # -> upper theta, lower theta
  def _get_servo_angle(p_table,s_global,servo_to_global):
    p_global = table_to_global(p_table, roll, tilt)
    # print(f"P_global = {P_global}")

    p_servo = global_to_servo(p_global, s_global, servo_to_global)
    # print(f"P_servo = {P_servo}")
    px = float(p_servo[0,0])
    py = float(p_servo[1,0])
    pz = float(p_servo[2,0])

    # theta = alpha +- arccos(k/sqrt(Py^2 + Px^2))
    k = (px*px + py*py + pz*pz + L1*L1 - L2*L2) / (2*L1)
    alpha = np.arctan2(py, px)
    r = np.hypot(px, py)
    c = k / r
    # print("\n--- DEBUG ---")
    # print("Px,Py,Pz =", px, py, pz)
    # print("r =", r, "k =", k, "c=k/r =", c)
    # print("S_global.T =", s_global.T)
    # print("P_global.T =", p_global.T)
    # print("P_servo.T  =", p_servo.T)
    if abs(c) > 1:
      raise ValueError("Unreachable geometry: |k/r| > 1 (no real IK solution).")
    delta = np.arccos(c)
    theta_a = alpha + delta
    theta_b = alpha - delta
    return theta_a, theta_b
  thetas = np.zeros((2,1),float)
  i = 1
  thetas[0][0] = _get_servo_angle(ROLL_ANCHOR_TABLE_COORDS,ROLL_SERVO_GLOB_COORDS,ROLL_SERVO_TO_GLOBAL)[1]
  thetas[1][0] = _get_servo_angle(TILT_ANCHOR_TABLE_COORDS,TILT_SERVO_GLOB_COORDS,TILT_SERVO_TO_GLOBAL)[0]
  return thetas


thetas_offset = get_servo_angles(roll=0.0, tilt=0.0)
thetas_phys = get_servo_angles(roll, tilt)
thetas_cmd = thetas_phys - thetas_offset        # so level becomes 0
# print(f"theta_cmd = {theta_cmd}")

def elbow_global(theta, S_global, R_servo_to_global):
    E_servo = np.array([[L1*np.cos(theta)],
                        [L1*np.sin(theta)],
                        [0.0]])
    return S_global + R_servo_to_global @ E_servo

roll_elbow_global_coords = elbow_global(thetas_phys[0][0],ROLL_SERVO_GLOB_COORDS,ROLL_SERVO_TO_GLOBAL)
print(f"S->E distance: {np.linalg.norm(roll_elbow_global_coords[:3] - ROLL_SERVO_GLOB_COORDS)}")
print(f"E->P distance: {round(np.linalg.norm(table_to_global(ROLL_ANCHOR_TABLE_COORDS, roll, tilt) - roll_elbow_global_coords[:3]),1)}")

print(f"\nPhysical Servo angles: {thetas_phys.T * 180 / np.pi} degrees.")
print(f"\nCommanded Servo_tilt angle: {thetas_cmd.T * 180/np.pi} degrees.")


import numpy as np
import matplotlib.pyplot as plt

# --- compute global points you care about ---
P_roll_global = table_to_global(ROLL_ANCHOR_TABLE_COORDS, roll, tilt)
P_tilt_global = table_to_global(TILT_ANCHOR_TABLE_COORDS, roll, tilt)

E_roll_global = elbow_global(thetas_phys[0,0], ROLL_SERVO_GLOB_COORDS, ROLL_SERVO_TO_GLOBAL)
E_tilt_global = elbow_global(thetas_phys[1,0], TILT_SERVO_GLOB_COORDS, TILT_SERVO_TO_GLOBAL)

# --- helper: equal-axis scaling for 3D ---
def set_equal_3d(ax, pts):
    pts = np.asarray(pts, float)
    xs, ys, zs = pts[:,0], pts[:,1], pts[:,2]
    xmid, ymid, zmid = xs.mean(), ys.mean(), zs.mean()
    max_range = max(np.ptp(xs), np.ptp(ys), np.ptp(zs)) / 2.0
    if max_range == 0:
        max_range = 1.0
    ax.set_xlim(xmid - max_range, xmid + max_range)
    ax.set_ylim(ymid - max_range, ymid + max_range)
    ax.set_zlim(zmid - max_range, zmid + max_range)

# --- collect points ---
points = {
    "Origin": ORIGIN,
    "Roll servo S_r": ROLL_SERVO_GLOB_COORDS,
    "Roll elbow E_r": E_roll_global,
    "Roll anchor P_r": P_roll_global,
    "Tilt servo S_t": TILT_SERVO_GLOB_COORDS,
    "Tilt elbow E_t": E_tilt_global,
    "Tilt anchor P_t": P_tilt_global,
}

# --- plot ---
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

for name, v in points.items():
    x, y, z = float(v[0,0]), float(v[1,0]), float(v[2,0])
    ax.scatter([x], [y], [z])
    ax.text(x, y, z, name, fontsize=9)

# link segments: S->E and E->P for each servo
def seg(a, b, label=None):
    ax.plot([a[0,0], b[0,0]], [a[1,0], b[1,0]], [a[2,0], b[2,0]], label=label)

seg(ROLL_SERVO_GLOB_COORDS, E_roll_global, "Roll horn S→E")
seg(E_roll_global, P_roll_global, "Roll rod E→P")
seg(TILT_SERVO_GLOB_COORDS, E_tilt_global, "Tilt horn S→E")
seg(E_tilt_global, P_tilt_global, "Tilt rod E→P")

# optional: draw global axes for orientation
axis_len = RADIUS * 0.6
ax.plot([0, axis_len], [0, 0], [0, 0], label="+X")
ax.plot([0, 0], [0, axis_len], [0, 0], label="+Y")
ax.plot([0, 0], [0, 0], [0, axis_len], label="+Z")

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.legend()

# equal scaling
pts_xyz = np.array([[float(v[0,0]), float(v[1,0]), float(v[2,0])] for v in points.values()])
set_equal_3d(ax, pts_xyz)

plt.show()
