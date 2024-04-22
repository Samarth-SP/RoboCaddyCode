import RPi.GPIO as GPIO
import time
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

def Power(direction, pin1, pin2):
    if direction:
        GPIO.output(pin1, GPIO.HIGH)
    else:
        GPIO.output(pin2, GPIO.HIGH)
    Reset(pin1, pin2)

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

def Intake():
    GPIO.output(16, GPIO.HIGH)
    GPIO.output(13, GPIO.HIGH)
Drive(True, 5)
Intake()
time.sleep(0.5)
Reset(16, 20)
Reset(19, 13)
GPIO.cleanup()