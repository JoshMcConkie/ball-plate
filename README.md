## TODO:
  1. Basic function
    - IMU
      - Code/testing
      - integrate in code
      - integrate on board
      - get feed of boards position
    - Camera
      - Lighting conditions
      - 

# Structure
BallMeasurement → estimator → BallState
IMUReading      → estimator → TableState
SystemState     → controller → ControlCommand
## Perception

-> Get ball coords
    - Camera feed
    - Lock px to x,y map (stable camera)
    - Get current x,y
    - Kalman Filter
-> Get table angle
    - Read IMU data

## Planning
-> Get desired position (PID)
-> convert to desired table angle
-> convert to servo angles

## Control
-> send servo commands over serial