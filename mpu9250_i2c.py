#!/usr/bin/python3 -u
#adapted from https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer


import smbus,time


# registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU6050_start():
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Write to general Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000, 0b010000, 0b10000, 0b11000] # byte registers
    gyro_config_vals = [250.0, 500.0, 1000.0, 2000.0] # °/sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000, 0b01000, 0b10000, 0b11000] # byte registers
    accel_config_vals = [2.0, 4.0, 8.0, 16.0] # g
    accel_indx = 0
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx]/(1<<15), accel_config_vals[accel_indx]/(1<<15)

def read_int16(register):
    # read accel and gyro values
    msb = bus.read_byte_data(MPU6050_ADDR, register)
    lsb = bus.read_byte_data(MPU6050_ADDR, register+1)
    value = ((msb<<8)|lsb)

    # 2-complement
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration values
    acc_x = read_int16(ACCEL_XOUT_H)
    acc_y = read_int16(ACCEL_YOUT_H)
    acc_z = read_int16(ACCEL_ZOUT_H)

    # raw gyroscope values
    gyro_x = read_int16(GYRO_XOUT_H)
    gyro_y = read_int16(GYRO_YOUT_H)
    gyro_z = read_int16(GYRO_ZOUT_H)

    # convert to g and °/s
    a_x = acc_x*accel_sens
    a_y = acc_y*accel_sens
    a_z = acc_z*accel_sens

    w_x = gyro_x*gyro_sens
    w_y = gyro_y*gyro_sens
    w_z = gyro_z*gyro_sens

    return a_x,a_y,a_z,w_x,w_y,w_z


# let's go
bus = smbus.SMBus(1)
gyro_sens, accel_sens = MPU6050_start() # setup acc+gyro

time.sleep(1) # let time for sensors to start

