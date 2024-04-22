import RPi.GPIO as GPIO
import smbus2
import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
##Motor1
GPIO.setup(14, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.HIGH)
##Motor2
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.output(25, GPIO.HIGH)
##Motor3
GPIO.setup(19, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, GPIO.HIGH)
##Motor4
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.HIGH)

CAMERA_HEIGHT = 9
FRAME_WIDTH = 1
FRAME_HEIGHT = 1
PIXEL_WIDTH = 1920
PIXEL_HEIGHT = 1080

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

OFFSET_Z = 0

bus = smbus2.SMBus(1)
Device_address = 0x68

#Camera Setup
camera = Picamera2()
camera.preview_configuration.main.size=(1920, 1080)
camera.preview_configuration.main.format="RGB888"
camera.preview_configuration.controls.FrameRate = 30
camera.configure('preview')
camera.start()

def MPU_Init():
    bus.write_byte_data(Device_address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_address, CONFIG, 0)
    bus.write_byte_data(Device_address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_address, addr)
    low = bus.read_byte_data(Device_address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value-65536
    return value

def Reset(pin1, pin2):
    GPIO.output(pin1, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    print('power off')

def Drive(direction, duration):
    if not direction:
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
    else:
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
    time.sleep(duration)
    Reset(14, 15)
    Reset(23, 24)
def Turn(direction):
    if direction:
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
    else:
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
def Intake():
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(13, GPIO.HIGH)
def GyroTurn(direction, degree):
    angleZ = 0
    Turn(direction)
    while abs(angleZ) < abs(degree):
        gyroz = read_raw_data(GYRO_ZOUT_H)-OFFSET_Z
        angleZ += 0.01*gyroz/131.0*8.57
        print(angleZ)
        time.sleep(0.01)
    Reset(14, 15)
    Reset(23, 24)
MPU_Init()
startTime = time.time()
gyroZsum = 0
while time.time() < startTime+15:
    gyroZsum += read_raw_data(GYRO_ZOUT_H)
    time.sleep(0.01)
OFFSET_Z = gyroZsum/1500.0

for i in range(12):
#Camera Frame Preview
    time.sleep(1)
    frame = camera.capture_array()
    hsvFrame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)   #Convert from BGR to HSV
    hsvFrameBlur = cv2.medianBlur(hsvFrame, 5)
    lower_green = np.array([20,40,60])
    upper_green = np.array([102,255,255])
#Filter for Green
    green_mask = cv2.inRange(hsvFrameBlur, lower_green, upper_green)   #Threshold HSV image to identify only Red
    result= cv2.bitwise_and(frame,frame, mask=green_mask) #Bitwise-AND original image with mask
    result_gray = cv2.split(result)[2]
#find all circles
    circles = cv2.HoughCircles(result_gray, cv2.HOUGH_GRADIENT, 1, 1000, param1=40, param2=30, minRadius=40, maxRadius=100)
    print(circles)
    if circles is not None:
        cv2.imwrite("circleimg.jpg",result_gray)
        circles = np.uint16(np.around(circles))
#write circles onto circle overlay file
        for i in circles[0,:]:
            print(i[0], i[1], i[2])
            centercoords = (i[0], i[1])
            radius = i[2]
            color = (0, 255, 0)
            cv2.circle(frame, centercoords, radius, color, 10)
#Angle Math, given (x, y) find angle, constants CAMERA_HEIGHT, BASE_DIST, FRAME_WIDTH, FRAME_HEIGHT, PIXEL_WIDTH, PIXEL_HEIGHT
            angle1 = 50*(i[0]-960)/1920
            angle2= 180/math.pi*np.arctan(((i[0]-960.0)/1920.0*(24.0+86.0*i[1]/1080.0))/(27.5+100.0*i[1]/1080.0))
            cv2.imwrite("circularoverlay.jpg", frame)
            print(angle1)
            print(angle2)
        GyroTurn(not(angle1>0), angle1)
        Intake()
        Drive(True, 6)
        time.sleep(2)
        Reset(16, 20)
        Reset(19, 13)
        break
    else:
        GyroTurn(True, 360.0/12)

GPIO.cleanup()
