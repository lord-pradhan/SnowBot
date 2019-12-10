"""
Program:     IMU_Class_Testing
gfRevised On:  11/21/2019
"""

### Library Imports
from MPU6050 import MPU6050
from time import sleep
from sys import exit
from signal import signal, SIGINT
###


### CTRL + C Signal Handler & Resource Cleanup
def signal_handler(sig, frame):
    """Handler for CTRL + C clean exit."""
    print('Quitting program.')
    cleanup()

def cleanup():
    """Resource cleanup."""
    imu.close()
    print('Resource cleanup completed.')
    exit(0)

signal(SIGINT, signal_handler)
###


### IMU Configuration
imu = MPU6050(ad0_bit=0)
imu.initialize()
imu.set_digital_filter(0)
rate = imu.set_sample_rate(100)
imu.set_accel_range(0)
imu.set_gyro_range(0)
###


### Main Program
print('Press CTRL + C to exit.')
print('Internal sampling rate: ' + str(rate) + ' Hz')

is_connected = imu.test_connection()
print('Connection test result ' + str(is_connected))

while True:
    imu.update_all()
    print('Accelerometer:\t' + str(imu.accel_x) + '\t' + str(imu.accel_y) + '\t' + str(imu.accel_z))
    print('Gyroscope:    \t' + str(imu.gyro_x) + '\t' + str(imu.gyro_y) + '\t' + str(imu.gyro_z) + '\n')
    sleep(1)
###
