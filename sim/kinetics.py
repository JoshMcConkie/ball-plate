import numpy as np
import matplotlib.pyplot as plt

L_1 = 2           # Length of servo arm (biceo)
L_2 = 6           # Length of tie rod (forearm)
RADIUS = 30       # 1/2 of tabletop width
x_motor = L_1     # Global pos of servo head
y_motor = RADIUS  # ...
z_motor = -L_2    # ...

max_theta = np.arccos(L_1/(L_1+L_2))  # Maximum c-clckws servo angle
print(f"max_theta = {max_theta * 180/np.pi} degrees")
theta = max_theta
delta_x = L_1 - L_1 * np.cos(theta)
delta_y = np.sqrt(L_2**2 - delta_x**2)
phi = np.atan2(delta_y,delta_x)-theta

ORIGIN = np.array([[0],[0],[0],[1]])  # Center pivot of table

# Homogeneous transformation matrices
# Tm: Global <- Motor
Tm = np.array([[-1,0,0,x_motor]
               ,[0,0,1,y_motor]
               ,[0,1,0,z_motor]
               ,[0,0,0,1]])
# Ts: Motor <- Shoulder angle
Ts = np.array([[np.cos(theta),-np.sin(theta), 0,0]
               ,[np.sin(theta),np.cos(theta),0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])
# T2e: Shoulder <- Elbow
T2e = np.array([[1,0,0,L_1]
                ,[0,1,0,0]
                ,[0,0,1,0]
                ,[0,0,0,1]])
# Te: shoulder Basis <- Elbow basis
Te = np.array([[np.cos(phi),-np.sin(phi), 0,0]
               ,[np.sin(phi),np.cos(phi),0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])
# Tc: Elbow <- final point (elbow basis)
Tc = np.array([[1,0,0,L_2]
               ,[0,1,0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])


C = Tm @ Ts @ T2e @ Te @ Tc @ ORIGIN

U_roll = np.arctan(C[2]/C[1])
np.set_printoptions(linewidth=np.inf, suppress=True)

print(f"delta_x = {delta_x}")
print(f"delta_y = {delta_y}")

print(f"phi = {phi * 180/np.pi} degrees")
print(f"theta = {theta * 180/np.pi} degrees"
)

print(f"Position = {np.round(C[:3],1)}")
print(f"Magnitude of C = {np.linalg.norm(C)}")
print(f"U_roll = {round(U_roll[0] * 180/np.pi,2)} degrees ")

if (__name__ == '__main__'):
    pass