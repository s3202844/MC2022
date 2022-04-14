import os
import cv2

if not os.path.exists("results/"):
    os.mkdir("results/")

cap = cv2.VideoCapture("test/0.avi")

i = 0
while(True):
    ret, frame = cap.read()
    if ret:
        cv2.rectangle(frame, (200, 200), (400, 400), (0, 255, 0))
        cv2.imshow("frame", frame)
        cv2.imwrite("results/"+str(i)+".png", frame)
        i += 1
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()