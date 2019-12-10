"""
Program:     ArduinoMegaI2C.py
Revised On:  11/21/2019
"""

### Library Imports
import pigpio
###


### Class Definition
class ArduinoMegaI2C:
    
    def __init__(self, addr, i2c_channel=1):
        self.ch = i2c_channel
        self.addr = addr
        
        (self.pi, self.handle) = self.__open_i2c()
    
    def __open_i2c(self):
        pi = pigpio.pi()
        try:
            handleIMU = pi.i2c_open(self.ch, self.addr)
            return (pi, handleIMU)
        except:
            print('I2C open failed.')
            return (-1, -1)
    
    def __write_bytes(self, data):
        self.pi.i2c_write_device(self.handle, data)
    
    def __read_bytes(self, numBytes):
        (count, dataByteArray) = self.pi.i2c_read_device(self.handle, numBytes)
        if(count > 1):
            return (count, list(dataByteArray))
        else:
            return (count, list(dataByteArray))
    
    def set_velocities(self, data):
##        data = [vel1, vel2, vel3, vel4]
        self.__write_bytes(data)
    
    def get_velocities(self):
        (count, data) = self.__read_bytes(4)
        print(count)
        if(count == 4):
            return data
        else:
            return [-999, -999, -999, -999]
    
    def close(self):
        self.pi.i2c_close(self.handle)
        self.pi.stop()
###
