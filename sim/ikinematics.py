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

roll = 0 * np.pi / 180 # rotation about x-axis
tilt = 6 * np.pi / 180 # rotation about y-axis

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
    R = Ry(-tilt) @ Rx(-roll)
    return O + R @ P_table


S_global = np.array([[RADIUS]
                     ,[L1]
                     ,[-L2]])
R_stog_q1 = np.array([[0,0,1]
                     ,[1,0,0]
                     ,[0,1,0]])


def global_to_servo(P_global, S_global, R_servo_to_global):
    R_glob_to_serv = R_servo_to_global.T
    return R_glob_to_serv @ (P_global - S_global)

'''
get_servo_angle: {Table point (T basis),roll,tilt} -> {Servo angle}
'''

def get_servo_angle(P_table, roll, tilt): # -> upper theta, lower theta
  P_global = table_to_global(P_table, roll, tilt)
  # print(f"P_global = {P_global}")
  P_servo = global_to_servo(P_global, S_global, R_stog_q1)
  # print(f"P_servo = {P_servo}")
  Px = float(P_servo[0,0])
  Py = float(P_servo[1,0])
  Pz = float(P_servo[2,0])

  # theta = alpha +- arccos(k/sqrt(Py^2 + Px^2))
  k = (Px*Px + Py*Py + Pz*Pz + L1*L1 - L2*L2) / (2*L1)
  alpha = np.arctan2(Py, Px)
  c = k/np.sqrt(Py * Py + Px * Px)
  if abs(c) > 1:
    raise ValueError("Unreachable geometry: |k/r| > 1 (no real IK solution).")
  delta = np.arccos(c)
  theta_a = alpha + delta
  theta_b = alpha - delta
  return theta_a, theta_b


theta_offset = get_servo_angle(P_T, roll=0.0, tilt=0.0)[0]
theta_phys = get_servo_angle(P_T, roll, tilt)[0]
theta_cmd = theta_phys - theta_offset        # so level becomes 0
# print(f"theta_cmd = {theta_cmd}")

T = np.array([[0,0,1,RADIUS]
                  ,[1,0,0,L1]
                  ,[0,1,0,-L2]
                  ,[0,0,0,1]])
E_global = T @ [[L1 * np.cos(theta_phys)]
         ,[L1 * np.sin(theta_phys)]
         ,[0]
         ,[1]]
# print(f"E_global = {E}")

print(f"S->E distance: {np.linalg.norm(E_global[:3] - S_global)}")
print(f"E->P distance: {round(np.linalg.norm(table_to_global(P_T, roll, tilt) - E_global[:3]),1)}")

P_GLOBAL = table_to_global(P_T, roll, tilt)
print(f"P_GLOBAL = {P_GLOBAL.T}")

print(f"Commanded Servo_tilt angle: {theta_cmd}")
