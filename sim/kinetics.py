import numpy as np
import matplotlib.pyplot as plt

'''
Purpose:
    Modeling one of two servos, that which cause rotation
    around the x-axis and positioned at the maximum of the
    y-axis. The servo arm is oriented to be horizontal, 
    pointing in the negative x-direction at theta == 0 degrees.

    Theta: angle of servo
    Phi: Angle of tierod relative to servo arm.


Current assumptions/TODO:
- P is locked on its y value. Meaning, when the table rolls,
the tie rod joint on the tabletop does not roll. Obviously
this is incorrect. Meant to simplify problem for now.


'''
L_1 = 2           # Length of servo arm (biceo)
L_2 = 6           # Length of tie rod (forearm)
RADIUS = 30       # 1/2 of tabletop width
x_motor = L_1     # Global pos of servo head
y_motor = RADIUS  # ...
z_motor = -L_2    # ...

max_theta = np.arccos(L_1/(L_1+L_2))    # Maximum ccwise servo angle (bicep)
print(f"max_theta = {max_theta * 180/np.pi} degrees")

theta = max_theta                       # Test sample angle
delta_x = L_1 - L_1 * np.cos(theta)     # positive x-distance from P joint to elbow
delta_y = np.sqrt(L_2**2 - delta_x**2)  # positive y-distance from P joint to elbow
phi = np.atan2(delta_y,delta_x)-theta   # ccwise angle of tie rod (forearm)

ORIGIN = np.array([[0],[0],[0],[1]])  # Center pivot of table

# Homogeneous transformation matrices
# Tm: Global <- Actuator
Ta2g = np.array([[-1,0,0,x_motor]
               ,[0,0,1,y_motor]
               ,[0,1,0,z_motor]
               ,[0,0,0,1]])
# Ts2s: Actuator <- Shoulder angle
Ts2a = np.array([[np.cos(theta),-np.sin(theta), 0,0]
               ,[np.sin(theta),np.cos(theta),0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])
# Te: shoulder basis/position <- Elbow basis/position
Te2s = np.array([[np.cos(phi),-np.sin(phi), 0,L_1]
               ,[np.sin(phi),np.cos(phi),0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])
# Tp: Elbow <- final point (elbow basis)
Tp2e = np.array([[1,0,0,L_2]
               ,[0,1,0,0]
               ,[0,0,1,0]
               ,[0,0,0,1]])


P = Ta2g @ Ts2a @ Te2s @ Tp2e @ ORIGIN

U_roll = np.arctan(P[2]/P[1])
np.set_printoptions(linewidth=np.inf, suppress=True)

print(f"delta_x = {delta_x}")
print(f"delta_y = {delta_y}")

print(f"phi = {phi * 180/np.pi} degrees")
print(f"theta = {theta * 180/np.pi} degrees"
)

print(f"Position = {np.round(P[:3],1)}")
print(f"Magnitude of P = {np.linalg.norm(P)}")
print(f"U_roll = {round(U_roll[0] * 180/np.pi,2)} degrees ")

if (__name__ == '__main__'):
    pass