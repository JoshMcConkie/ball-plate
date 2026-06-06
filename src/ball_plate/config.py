from ball_plate.state import ReferenceState, TableState

BAUD_RATE = 115200
SERIAL_PORT = 'COM3'

CAM_ID = 0

REFERENCE_STATE = ReferenceState(timestamp=0.0,x_goal=0.0,y_goal=0.0)
CAMERA_HZ = 60
IMU_HZ = 100
STATE_EST_HZ = 100
CONTROL_HZ = 50
SERIAL_HZ = 50
DEBUG_HZ = 5

