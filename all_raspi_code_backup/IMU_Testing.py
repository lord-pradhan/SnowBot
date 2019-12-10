"""
Program:     IMU_Testing
Revised On:  11/21/2019
"""

### Library Imports
import pigpio
from time import sleep
from sys import exit
from signal import signal, SIGINT
###


### IMU Registers
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
###


### CTRL + C Signal Handler & Resource Cleanup
def signal_handler(sig, frame):
    """Handler for CTRL + C clean exit."""
    print('Quitting program.')
    cleanup()

def cleanup():
    """Resource cleanup."""
    pi.i2c_close(handle_IMU)
    pi.stop()
    print('Resource cleanup completed.')
    exit(0)

signal(SIGINT, signal_handler)
###


### Function Definitions
def accelWordToFloat(word):
    """Converts the 16-bit 2's-complement ADC word into appropriate float.
    
    Division scaling assumes +-2g full-scale range.
    
    Args:
        word - 16-bit 2's-complement ADC word.
    Return:
        float
    """
    if word & 0x8000:
        return float((word ^ 0xffff) + 1) / 16384 * -1
    else:
        return float(word) / 16384

def gyroWordToFloat(word):
    """Converts the 16-bit 2s-complement ADC word into appropriate float.
    
    Division scaling assumes +-250deg/s full-scale range.
    
    Args:
        word - 16-bit 2's-complement ADC word.
    Return:
        float
    """
    if word & 0x8000:
        return float((word ^ 0xffff) + 1) / 131 * -1
    else:
        return float(word) / 131


### I2C Setup
# Set I2C channel
channel = 1

# IMU 7-bit address
IMU_ADDR = 0b1101000

# Initialize I2C through pigpio
pi = pigpio.pi()
try:
    handle_IMU = pi.i2c_open(channel, IMU_ADDR)
except:
    # Exit program if I2C open error occurs
    print('I2C open error occurred.')
    exit(0)
###


### IMU Configuration
# Single-Byte Write:  pi.i2c_write_byte_data(handle, reg, data)
# Burst Write:        pi.i2c_write_i2c_block_data(handle, reg, data)
# Single-Byte Read:   pi.i2c_read_byte_data(handle, reg)
# Burst Read:         pi.i2c_read_i2c_block_data(handle, reg, count)
pi.i2c_write_byte_data(handle_IMU, REG_PWR_MGMT_1, 0b10000000)
sleep(0.1)
pi.i2c_write_byte_data(handle_IMU, REG_PWR_MGMT_1, 0b00001000)
pi.i2c_write_byte_data(handle_IMU, REG_PWR_MGMT_2, 0b00000000)
pi.i2c_write_byte_data(handle_IMU, REG_CONFIG, 0b00000000)
pi.i2c_write_byte_data(handle_IMU, REG_SAMP_RATE_DIV, 79)
pi.i2c_write_byte_data(handle_IMU, REG_GYRO_CONFIG, 0b00000000)
pi.i2c_write_byte_data(handle_IMU, REG_ACCEL_CONFIG, 0b00000000)
###


### Main Program
print('Press CTRL + C to exit.')

ba = pi.i2c_read_byte_data(handle_IMU, REG_WHO_AM_I)
print('Address: ' + str(ba))

values = [5, 6]
while True:
    ba = pi.i2c_read_byte_data(handle_IMU, REG_ACCEL_X_H)
    values[0] = ba
    ba = pi.i2c_read_byte_data(handle_IMU, REG_ACCEL_X_L)
    values[1] = ba
    
    accelX = ((values[0] << 8) | values[1])
    print(accelX)
    
    sleep(1)
###
