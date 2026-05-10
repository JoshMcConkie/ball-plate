import cv2

from ball_plate import perception

feed = perception.init_camera()
ret0, frame0 = feed.read()
filt_init_frame = perception.filter_frame(frame0)
while True:
    ret, frame = feed.read()
    if not ret:
        break

    # ==Perception==
    mask = perception.get_mask(filt_init_frame, frame)
    x,y = perception.get_position(mask)

    # ==Planning==


    #==Control==


    cv2.imshow("Webcam Feed", mask)
    if cv2.waitKey(1) == 27: # Esc key exit
        break
feed.release()
cv2.destroyAllWindows()