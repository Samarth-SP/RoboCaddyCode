import smbus2
import time

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

OFFSET_X = 0
OFFSET_Y = 0
OFFSET_Z = 0

bus = smbus2.SMBus(1)
Device_address = 0x68

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
    
MPU_Init()
startTime = time.time()
gyroZsum = 0
while time.time() < startTime+15:
    gyroZsum += read_raw_data(GYRO_ZOUT_H)
    time.sleep(0.01)
OFFSET_Z = gyroZsum/1500.0
angleZ=0
while True:
    gyroz = read_raw_data(GYRO_ZOUT_H)-OFFSET_Z
    angleZ += 0.01*gyroz/131.0*10.59
    print(angleZ)
    time.sleep(0.01)