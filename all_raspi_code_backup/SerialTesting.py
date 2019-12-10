"""
Program:     SerialTesting.py
Revised On:  12/04/2019
"""

### Library Imports
import serial
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
    serialDevice.close()
    print('Resource cleanup completed.')
    exit(0)

signal(SIGINT, signal_handler)
###


### Serial Setup
BAUD = 9600
serialDevice = serial.Serial(port='/dev/ttyS0', baudrate=BAUD)
serialDevice.timeout = 2.0
###


### Main Program
print('Press CTRL + C to exit.')

counter = 1
while True:
    serialDevice.write(b'Testing ')
    serialDevice.write([counter])
    serialDevice.write(b'\n')
    print('Serial data sent to PC.')
    
    sleep(1.0)
###
