"""
Program:     MPU6050.py
Revised On:  11/21/2019
"""

### Library Imports
import pigpio
import time
from time import sleep
from usefulFuncs import *
##from brain import *
##from classDefs import *
##from DriveArduino import *
import numpy as np
###


### Class Definition
class MPU6050:
    
    # MPU-6050 Registers
    REG_SAMP_RATE_DIV = 25
    REG_CONFIG = 26
    REG_GYRO_CONFIG = 27
    REG_ACCEL_CONFIG = 28
    REG_FIFO_EN = 35
    REG_ACCEL_X_H = 59
    REG_ACCEL_X_L = 60
    REG_ACCEL_Y_H = 61
    REG_ACCEL_Y_L = 62
    REG_ACCEL_Z_H = 63
    REG_ACCEL_Z_L = 64
    REG_GYRO_X_H = 67
    REG_GYRO_X_L = 68
    REG_GYRO_Y_H = 69
    REG_GYRO_Y_L = 70
    REG_GYRO_Z_H = 71
    REG_GYRO_Z_L = 72
    REG_PWR_MGMT_1 = 107
    REG_PWR_MGMT_2 = 108
    REG_FIFO_CNT_H = 114
    REG_FIFO_CNT_L = 115
    REG_FIFO_DATA = 116
    REG_WHO_AM_I = 117
    
    # I2C 7-bit base address
    ADDR_BASE = 0b1101000
    
    def __init__(self, theta_init, i2c_channel=1, ad0_bit=0):
        self.ch = i2c_channel
        self.addr = self.ADDR_BASE if not ad0_bit else (self.ADDR_BASE | 1)
        
        (self.pi, self.handle) = self.__open_i2c()
        
        self.fs = 8000
        self.accel_fsr = 0
        self.gyro_fsr = 0
        
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.yaw = theta_init
        self.sum_yaw_rate = 0
        self.ct_yaw = 0
        self.time = int(round(time.time()))
        self.yawZero = 0.018
    
    def __open_i2c(self):
        pi = pigpio.pi()
        try:
            handle = pi.i2c_open(self.ch, self.addr)
            return (pi, handle)
        except:
            print('I2C open failed.')
            return (-1, -1)
    
    def __write_byte(self, reg, data):
        while(1):
            try:
                self.pi.i2c_write_byte_data(self.handle, reg, data)
                break
            except:
                print('i2c write MPU try-catch')
                continue
    
    def __read_byte(self, reg):
        while(1):
            try:
                return self.pi.i2c_read_byte_data(self.handle, reg)
            except:
                print('i2c read MPU try-catch')
                continue
    
    def initialize(self):
        # Reset IMU.
        self.__write_byte(self.REG_PWR_MGMT_1, 0b10000000)
        sleep(0.1)
        # Wake IMU, disable sleep, disable cycling, disable temperature sensor,
        # select 8MHz internal oscillator, 
        self.__write_byte(self.REG_PWR_MGMT_1, 0b00001000)
        self.__write_byte(self.REG_PWR_MGMT_2, 0b00000000)
        # Disable external sync, disable digital low-pass filter.
        self.__write_byte(self.REG_CONFIG, 0b00000000)

    def set_digital_filter(self, config):
        """Configure digital low-pass filter for accelerometer and gyroscope.
        
        Args:
            config - DLPF configuration value. Valid values:
                0 - (accel) 260Hz  0.0ms delay, (gyro) 256Hz  0.98ms delay
                1 - (accel) 184Hz  2.0ms delay, (gyro) 188Hz  1.90ms delay
                2 - (accel)  94Hz  3.0ms delay, (gyro)  98Hz  2.80ms delay
                3 - (accel)  44Hz  4.9ms delay, (gyro)  42Hz  4.80ms delay
                4 - (accel)  21Hz  8.5ms delay, (gyro)  20Hz  8.30ms delay
                5 - (accel)  10Hz 13.8ms delay, (gyro)  10Hz 13.40ms delay
                6 - (accel)   5Hz 19.0ms delay, (gyro)   5Hz 18.60ms delay
                7 - reserved
        """
        data = config & 0x07
        self.__write_byte(self.REG_CONFIG, data)
        if (data == 0) or (data == 7):
            self.fs = 8000
        else:
            self.fs = 1000
    
    def set_sample_rate(self, rate):
        """Set the sample rate divider.
        
        Args:
            rate - Desired sampling rate, in Hz.
        Returns:
            float - Actual sampling rate, in Hz.
        """
        data = int(self.fs / rate - 1) & 0xff
        self.__write_byte(self.REG_SAMP_RATE_DIV, data)
        return self.fs / float(data + 1)
    
    def set_accel_range(self, fsr):
        """Set the accelerometer full-scale range.
        
        Args:
            fsr - Full-scale range configuration value. Valid values:
                0 - +- 2 g
                0 - +- 4 g
                0 - +- 8 g
                0 - +-16 g
        """
        self.accel_fsr = fsr & 0x03
        data = self.accel_fsr << 3
        self.__write_byte(self.REG_ACCEL_CONFIG, data)
    
    def set_gyro_range(self, fsr):
        """Set the gyroscope full-scale range.
        
        Args:
            fsr - Full-scale range configuration value. Valid values:
                0 - +- 250 deg/s
                0 - +- 500 deg/s
                0 - +-1000 deg/s
                0 - +-2000 deg/s
        """
        self.gyro_fsr = fsr & 0x03
        data = self.gyro_fsr << 3
        self.__write_byte(self.REG_GYRO_CONFIG, data)
    
    def update_all(self):
        """Read most recent sensor data."""
        accel_x_word = [0, 0]
        accel_y_word = [0, 0]
        accel_z_word = [0, 0]
        gyro_x_word = [0, 0]
        gyro_y_word = [0, 0]
        gyro_z_word = [0, 0]
        
        accel_x_word[0] = self.__read_byte(self.REG_ACCEL_X_H)
        accel_x_word[1] = self.__read_byte(self.REG_ACCEL_X_L)
        accel_y_word[0] = self.__read_byte(self.REG_ACCEL_Y_H)
        accel_y_word[1] = self.__read_byte(self.REG_ACCEL_Y_L)
        accel_z_word[0] = self.__read_byte(self.REG_ACCEL_Z_H)
        accel_z_word[1] = self.__read_byte(self.REG_ACCEL_Z_L)
        gyro_x_word[0] = self.__read_byte(self.REG_GYRO_X_H)
        gyro_x_word[1] = self.__read_byte(self.REG_GYRO_X_L)
        gyro_y_word[0] = self.__read_byte(self.REG_GYRO_Y_H)
        gyro_y_word[1] = self.__read_byte(self.REG_GYRO_Y_L)
        gyro_z_word[0] = self.__read_byte(self.REG_GYRO_Z_H)
        gyro_z_word[1] = self.__read_byte(self.REG_GYRO_Z_L)
        
        self.accel_x = self.__accel_word_to_float(accel_x_word)
        self.accel_y = self.__accel_word_to_float(accel_y_word)
        self.accel_z = self.__accel_word_to_float(accel_z_word)
        self.gyro_x = self.__gyro_word_to_float(gyro_x_word)
        self.gyro_y = self.__gyro_word_to_float(gyro_y_word)
        self.gyro_z = self.__gyro_word_to_float(gyro_z_word)
    
    def __accel_word_to_float(self, arr):
        """Converts the accelerometer 16-bit 2's-complement ADC word into appropriate float.    
        
        Args:
            arr - 16-bit 2's-complement ADC word (as length-2 array).
        Return:
            float
        """
        word = (arr[0] << 8) | arr[1]
        if word & 0x8000:
            word = -1 * ((word ^ 0xffff) + 1)
        if self.accel_fsr == 0:
            return float(word) / 16384.0
        elif self.accel_fsr == 1:
            return float(word) / 8192.0
        elif self.accel_fsr == 2:
            return float(word) / 4096.0
        else:
            return float(word) / 2048.0
    
    def __gyro_word_to_float(self, arr):
        """Converts the gyroscope 16-bit 2s-complement ADC word into appropriate float.
        
        Args:
            arr - 16-bit 2's-complement ADC word (as length-2 array).
        Return:
            float
        """
        word = (arr[0] << 8) | arr[1]
        if word & 0x8000:
            word = -1 * ((word ^ 0xffff) + 1)
        if self.gyro_fsr == 0:
            return float(word) / 131.0
        elif self.gyro_fsr == 1:
            return float(word) / 65.5
        elif self.gyro_fsr == 2:
            return float(word) / 32.8
        else:
            return float(word) / 16.4
    
    def test_connection(self):
        data = self.__read_byte(self.REG_WHO_AM_I)
        if data == (self.addr & 0xfe):
            return 0
        else: 
            return -1
    
    def close(self):
        self.pi.i2c_close(self.handle)
        self.pi.stop()

    def setup(self):
        ### IMU Configuration
        # imu = MPU6050(ad0_bit=0)
        self.initialize()
        self.set_digital_filter(0)
        rate = self.set_sample_rate(100)
        # imu.set_accel_range(0)
        self.set_gyro_range(0)
        ###

    def getYaw(self):
        self.update_all()
        yaw_rate = np.radians(self.gyro_z)
        self.sum_yaw_rate += yaw_rate
        self.ct_yaw += 1
        self.YRav = self.sum_yaw_rate /  self.ct_yaw
##        print('IMU charac', yaw_rate, self.YRav)
        currTime = int(round(time.time()))
        elapsedTime = ( currTime - self.time )
        self.time = currTime
        new_yaw = self.yaw + (yaw_rate+self.yawZero)*elapsedTime
        self.yaw = wrap2pi(new_yaw)

        return self.yaw
###

