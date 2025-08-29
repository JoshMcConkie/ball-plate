import cv2
import numpy as np
import time
import serial

'''Time constraints'''
last_send = 0.0
SEND_INTERVAL = 1.0/50.0 # aka 50 Hz


'''Serial'''
ser = serial.Serial('COM10', 115200, timeout=1)
time.sleep(2)
banner = ser.readline().decode(errors='ignore').strip()
print("Banner:", banner)

'''Tracking + Packet send'''
# Start the webcam
cap = cv2.VideoCapture(1)


cx_1 = None
cy_1 = None

vx = 0
vy = 0

prev_t = None

while True:
    # Read frame
    ret, frame = cap.read()

    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # My ball is green, change the below bounds for different color
    lower_green = np.array([40,70,70])
    upper_green = np.array([80, 255, 255])

    # binary mask
    mask = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask,None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if contours:

        c = max(contours, key=cv2.contourArea)


        # initial circle around contour
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

                # Only send packet if it follows the Hz constraint
                if (now - last_send) >= SEND_INTERVAL:
                    # build a packet
                    packet = f"{cx},{cy},{int(vx)},{int(vy)}\n"
                    try: # send/recieve from arduino through serial
                        ser.write(packet.encode('utf-8'))
                        echo = ser.readline().decode(errors='ignore').strip()
                        print("Echo:", echo)
                        last_send = now
                    except Exception as e:
                        print("Serial write failed:", e)
              
            else:
                print(f"Ball position: ({cx}, {cy}); Velocity (p/s): (N/A)")
                
            # draw for visual feedback
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            cx_1, cy_1 = cx, cy
            prev_t = now




    if not ret:
        break

    cv2.imshow("Webcam Feed", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()