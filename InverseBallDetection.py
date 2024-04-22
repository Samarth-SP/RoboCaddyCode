import numpy as np
import cv2
import RPi.GPIO as GPIO
from picamera2 import Picamera2
#constants declaration
CAMERA_HEIGHT = 9
FRAME_WIDTH = 1
FRAME_HEIGHT = 1
PIXEL_WIDTH = 1920
PIXEL_HEIGHT = 1080
# GPIO Warning Remove
GPIO.setwarnings(False)

#Camera Setup
camera = Picamera2()
camera.preview_configuration.main.size=(1920, 1080)
camera.preview_configuration.main.format="RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure('preview')
camera.start()

while(True):
#Camera Frame Preview
    frame = camera.capture_array()
    hsvFrame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   #Convert from BGR to HSV
    hsvFrameBlur = cv2.medianBlur(hsvFrame, 1)
    lenience = 30
    lower_green = np.array([20,40,60])
    upper_green = np.array([102,255,255])
#Filter for Green
    green_mask = cv2.inRange(hsvFrameBlur, lower_green, upper_green)   #Threshold HSV image to identify only Red
    result= cv2.bitwise_and(frame,frame, mask=green_mask) #Bitwise-AND original image with mask
    result_gray = cv2.split(result)[2]
    cv2.imshow("preview", result_gray)
#upon button press, capture image, close camera
    if cv2.waitKey(5) & 0xFF == ord('q'):
        camera.close()
        cv2.imwrite("circleimg.jpg",result_gray)
#find all circles
        circles = cv2.HoughCircles(result_gray, cv2.HOUGH_GRADIENT, 1, 1000, param1=40, param2=30, minRadius=0, maxRadius=100)
        print(circles)
        circles = np.uint16(np.around(circles))
#write circles onto circle overlay file
        for i in circles[0,:]:
            print(i[0], i[1], i[2])
            centercoords = (i[0], i[1])
            radius = i[2]
            color = (0, 255, 0)
            cv2.circle(frame, centercoords, radius, color, 10)
            #Angle Math, given (x, y) find angle, constants CAMERA_HEIGHT, BASE_DIST, FRAME_WIDTH, FRAME_HEIGHT, PIXEL_WIDTH, PIXEL_HEIGHT
            angle = 50*(i[0]-960)/1920
        cv2.imwrite("circularoverlay.jpg", frame)
        print(angle)
      



GPIO.cleanup()
