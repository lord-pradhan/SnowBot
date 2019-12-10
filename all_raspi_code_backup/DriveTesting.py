"""
Program:     DriveTesting.py
Revised On:  12/01/2019
"""

### Library Imports
from DriveArduino import DriveArduino
import numpy as np
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
    drive.close()
    print('Resource cleanup completed.')
    exit(0)

signal(SIGINT, signal_handler)
###


### Arduino Configuration
addr = 0x08
drive = DriveArduino(addr)
###


### Main Program
print('Press CTRL + C to exit.')

while True:
    setpoints = np.array([25, 25, -25, -25])
    drive.set_rpm(setpoints)
    sleep(1)
    
    drive.update()
    print(drive.rpm)
    print(drive.pwm)
    print()
###
