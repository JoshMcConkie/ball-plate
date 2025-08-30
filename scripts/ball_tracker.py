import cv2
import numpy as np
import time
import serial

SERIAL_ON = True
CAMERA_ID = 1

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
prev_t = None

# Start the webcam
feed = cv2.VideoCapture(CAMERA_ID)
ret0,frame0 = feed.read()

while True:
    # Read frame
    ret, frame = feed.read()
    if not ret:
        break
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # The ball in use is green. Change the below bounds for different color.
    lower_green = np.array([40,70,70])
    upper_green = np.array([80, 255, 255])

    # Binary mask (green:1 else:0)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask,None, iterations=2)
    # Identify the ball contour within the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if contours: # Prevents break if no green in frame

        c = max(contours, key=cv2.contourArea)

        # Initial circle around ball
        (x,y), radius = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

    
        # getting center point of contour
        if M["m00"] > 0 and radius > 8:
            now = time.time()
            cx = int(M["m10"] /M["m00"])
            cy = int(M["m01"] /M["m00"])
            if cx_1 is not None and cy_1 is not None and prev_t is not None: 
                dt = now - prev_t
                vx = (cx - cx_1)/dt
                vy = (cy - cy_1)/dt
                # print(f"Ball position: ({cx}, {cy}); Velocity (p/s): ({vx}, {vy})")

                # Only send packet if it follows the Hz constraint (50 Hz)
                # Packet Format: "float(x),float(y),int(vx),int(vy)"
                if SERIAL_ON and (now - last_send) >= SEND_INTERVAL:
                    # build a packet
                    packet = f"{cx},{cy},{int(vx)},{int(vy)}\n" #
                    try: # send/recieve from arduino through serial
                        ser.write(packet.encode('utf-8'))
                        echo = ser.readline().decode(errors='ignore').strip()
                        print("Echo:", echo)
                        last_send = now
                    except Exception as e:
                        print("Serial write failed:", e)
              
            else: # first frame rule
                print(f"Ball position: ({cx}, {cy}); Velocity (p/s): (N/A)")
                
            # Illustrate circle on out frame
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            cx_1, cy_1 = cx, cy
            prev_t = now

    cv2.imshow("Webcam Feed", frame)

    if cv2.waitKey(1) == 27: # Esc key exit
        break

feed.release()
cv2.destroyAllWindows()