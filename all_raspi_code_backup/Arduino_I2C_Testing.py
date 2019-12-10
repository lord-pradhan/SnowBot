"""
Program:     Arduino_I2C_Testing
Revised On:  11/21/2019
"""

### Library Imports
from ArduinoMegaI2C import ArduinoMegaI2C
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
    mega.close()
    print('Resource cleanup completed.')
    exit(0)

signal(SIGINT, signal_handler)
###


### Arduino Configuration
addr = 0x08
mega = ArduinoMegaI2C(addr)
###


### Main Program
print('Press CTRL + C to exit.')

while True:
    setpoints = [5, 5, 5, 5]
    mega.set_velocities(setpoints)
    sleep(1)
    
    encoderTicks = mega.get_velocities()
    print(encoderTicks[0]);
    print(encoderTicks[1]);
    print(encoderTicks[2]);
    print(encoderTicks[3]);
    print();
    f32 = encoderTicks[0] | (encoderTicks[1] << 8) | (encoderTicks[2] << 16) | (encoderTicks[3] << 24)
    f64 = ((f32 & 0x8000) << 32) | (((f32 & 0x7f800000) + 896) << 29) | ((f32 & 0x007fffff) << 29)
    f64_le = (f64 >> 56) | ((f64 >> 40) & 0xff00) | ((f64 >> 24) & 0xff0000) | ((f64 >> 8) & 0xff000000) | ((f64 & 0xff000000) << 8) | ((f64 & 0xff0000) << 24) | ((f64 & 0xff00) << 40) | ((f64 & 0xff) << 56)
    print(float(f64_le))
##    print('Velocities:\t' + str(encoderTicks[0]) + '\t' + str(encoderTicks[1]) + '\t' + str(encoderTicks[2]) + '\t' + str(encoderTicks[3]))
    
###
