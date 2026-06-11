from ball_plate.state import ReferenceState, TableState

BAUD_RATE = 115200
SERIAL_PORT = '/dev/ttyUSB0'

CAM_ID = 0

REFERENCE_STATE = ReferenceState(timestamp=0.0,x_goal=0.0,y_goal=0.0)
CAMERA_HZ = 60
IMU_HZ = 100
STATE_EST_HZ = 100
CONTROL_HZ = 50
SERIAL_HZ = 50
DEBUG_HZ = 2

COLOR_BALL = (0, 0, 255)

TABLE_W_M = 0.22  # table width in meters (x)
TABLE_H_M = 0.22  # table height in meters (y)

SERVO_ARM_LENGTH = 0.023  # servo arm length in meters