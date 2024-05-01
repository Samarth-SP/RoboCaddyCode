import RPi.GPIO as GPIO
import smbus2
import time
import math
import numpy as np
import cv2
from picamera2 import *
import tflite_runtime.interpreter as tflite

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
##Constants for Gyro Setup
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
GYRO_ZOUT_H = 0x47
##Gyro Offset Calculated Constant
OFFSET_Z = 0
##Declare Gyro Bus
bus = smbus2.SMBus(1)
Device_address = 0x68

##Declare Camera Frame Sizes
normS = (640, 480)
lowS = (320, 240)

##Initialize the Gyro Addresses (from documentation)
def MPU_Init():
    bus.write_byte_data(Device_address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_address, CONFIG, 0)
    bus.write_byte_data(Device_address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_address, INT_ENABLE, 1)
##define function to read data from Gyro (from documentation)
def read_raw_data(addr):
    high = bus.read_byte_data(Device_address, addr)
    low = bus.read_byte_data(Device_address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value-65536
    return value
#Function to reset Hbridge Pins to avoid damage
def Reset(pin1, pin2):
    GPIO.output(pin1, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    print('power off')
##Function to drive bot forward a given duration
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
##Function to turn bot
def Turn(direction):
    if direction:
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
    else:
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
##Function to run intake spinner motors
def Intake():
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(13, GPIO.HIGH)

##Function which integrates gyro acceleration values (roughly)
##and stops once turn angle is reached. angleZ holds angle.
def GyroTurn(direction, degree):
    angleZ = 0
    Turn(direction)
    ##while measured angle is less than target, keep turning
    while abs(angleZ) < abs(degree):
        gyroz = read_raw_data(GYRO_ZOUT_H)-OFFSET_Z
        angleZ += 0.01*gyroz/131.0*10.59
        print(angleZ)
        time.sleep(0.01)
    ##reset all motors once turn is achieved
    Reset(14, 15)
    Reset(23, 24)

##Function to read the labels from labelfile into an array (from documentation)
def ReadLabelFile(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    ret = {}
    for line in lines:
        pair = line.strip().split(maxsplit=1)
        ret[int(pair[0])] = pair[1].strip()
    return ret

##partially from picamera documentation, simplified to usecase
def InferenceTensorFlow(image, model):
    ##reads labels for object detection classes for the specific model from a text file
    labels = ReadLabelFile('coco_labels.txt')
    interpreter = tflite.Interpreter(model_path=model, num_threads=4)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    ##resize image based on model input parameters
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    rgb = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
    initial_h, initial_w, channels = rgb.shape
    picture = cv2.resize(rgb, (width, height))
    input_data = np.expand_dims(picture, axis=0)
    
    ##run tensor model on image
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    
    ##pull tensor array
    detected_boxes = interpreter.get_tensor(output_details[0]['index'])
    detected_classes = interpreter.get_tensor(output_details[1]['index'])
    detected_scores = interpreter.get_tensor(output_details[2]['index'])
    num_boxes = interpreter.get_tensor(output_details[3]['index'])
    rectangles = []
    for i in range(int(num_boxes)):
        top, left, bottom, right = detected_boxes[0][i]
        classId = int(detected_classes[0][i])
        ##filter all detected objects for only golf balls
        if classId == 36:
            if detected_scores[0][i] > 0.3:
                ##find center coordinates for golf ball
                centerx = (left*initial_w+right*initial_w)/2.0
                centery = (bottom*initial_h+top*initial_h)/2.0
                print(centerx, centery, labels[classId], 'score = ', detected_scores[0][i])
                ##calculate angle based on coordinate math
                angle2= 180/math.pi*np.arctan(((centerx-160.0)/320.0*(47.0+108.0*centery/240.0))/(18.65+112.7*centery/240.0))
                return angle2

def DetectionRuntime():
    for i in range(12):
        #Camera Frame Capture (sleep to stabilize frame)
        time.sleep(1)
        ##run tensorflow model on a low resolution stream buffer from the picamera
        buffer = camera.capture_buffer("lores")
        grey = buffer[:stride*lowS[1]].reshape((lowS[1], stride))
        result = InferenceTensorFlow(grey,'mobilenet_v2.tflite')
        ##result returns an angle to the ball, or None if there's nothing
        if result is not None:
            print(result)
            ##turn the specified angle, start intake, and drive forward 3s with intake on for 4s
            GyroTurn(not(result>0),result*0.2)
            Intake()
            Drive(True, 3)
            time.sleep(1)
            Reset(16, 20)
            Reset(19, 13)
            DetectionRuntime()
            break
        ##break loop once something is detected
        else:
            ##if nothing is detected, keep turning
            GyroTurn(True, 360.0/20)

##Main Runtime
MPU_Init()
startTime = time.time()
gyroZsum = 0
Reset(16, 20)
Reset(19, 13)
Reset(14, 15)
Reset(23, 24)
##Start a 15 second gyro calibration period
camera = Picamera2()
config = camera.create_preview_configuration(main={"size": normS}, lores={"size": lowS, "format": "YUV420"})
camera.configure(config)
stride = camera.stream_configuration("lores")["stride"]
camera.start()
while time.time() < startTime+15:
    gyroZsum += read_raw_data(GYRO_ZOUT_H)
    time.sleep(0.01)
OFFSET_Z = gyroZsum/1500.0
##Cover the 360 degree detection range in 12 turns
DetectionRuntime()
GPIO.cleanup()