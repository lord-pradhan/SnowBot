"""
Program:     IMUMag_Testing
gfRevised On:  12/07/2019
"""

### Library Imports
from ICM20948 import ICM20948
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
imu = ICM20948(ad0_bit=1)
imu.setup()
###


### Main Program
print('Press CTRL + C to exit.')

is_connected = imu.test_connection()
print('Connection test result ' + str(is_connected))

while True:
    imu.update_all()
##    print('Accel:\t' + str(imu.accel_x) + '\t' + str(imu.accel_y) + '\t' + str(imu.accel_z))
##    print('Gyro: \t' + str(imu.gyro_x) + '\t' + str(imu.gyro_y) + '\t' + str(imu.gyro_z))
    print('Mag:  \t' + str(imu.mag_x) + '\t' + str(imu.mag_y) + '\t' + str(imu.mag_z))
    print()
    sleep(0.2)
###
