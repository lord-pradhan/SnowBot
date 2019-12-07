"""
Program:     DriveArduino.py
Revised On:  12/01/2019
"""

### Library Imports
import pigpio
import numpy as np
##from brain import *
##from classDefs import *
##from MPU6050 import *
###


### Class Definition
class DriveArduino:

    ticks_per_rev = 160
    ints_per_tick = 2
    samp_period = 0.125
    
    def __init__(self, addr, i2c_channel=1):
        self.ch = i2c_channel
        self.addr = addr
        
        (self.pi, self.handle) = self._open_i2c()
        self.rpm_to_enc = self.ticks_per_rev * self.ints_per_tick * self.samp_period / 60.0

        self.rpm = np.array([0, 0, 0, 0])
        self.pwm = np.array([0, 0, 0, 0])
    
    def _open_i2c(self):
        pi = pigpio.pi()
        try:
            handleIMU = pi.i2c_open(self.ch, self.addr)
            return (pi, handleIMU)
        except:
            print('I2C open failed.')
            return (-1, -1)
    
    def _write_bytes(self, data):
        while(1):
            try:
                self.pi.i2c_write_device(self.handle, data)
                break
            except:
                print('i2c write try except triggered')
                continue
                                
    
    def _read_bytes(self, numBytes):
        while(1):
            try:
                                    
                (count, dataByteArray) = self.pi.i2c_read_device(self.handle, numBytes)
                if(count > 1):
                    return (count, list(dataByteArray))
                else:
                    return (count, dataByteArray)
                break
            except:
                print('i2c read arduino try except trigerred')
                continue
    
    def set_rpm(self, rpm_arr):
        """Set angular velocity for each wheel in RPM.

        Args:
            rpm_arr - Numpy array of angular velocities in RPM.
        """
        enc_arr = np.around(rpm_arr * self.rpm_to_enc).astype('int8')
        #print('Arduino I2C: ', rpm_arr * self.rpm_to_enc)
        data = np.bitwise_and(enc_arr, 0xff).tolist()
        self._write_bytes(data)
    
    def update(self):
        (count, data) = self._read_bytes(12)
        print(data)
        self._process_enc_bytes(data[0:4])
        self._process_pwm_bytes(data[4:])

    def _process_enc_bytes(self, enc_bytes):
        for i in range(4):
            enc_value = int(enc_bytes[i])
            print(enc_value)
            if enc_value & 0x80:
                enc_value = -1 * ((enc_value ^ 0xff) + 1)
            self.rpm[i] = float(enc_value) / self.rpm_to_enc

    def _process_pwm_bytes(self, pwm_bytes):
        for i in range(4):
            pwm_value = int(pwm_bytes[2*i]) | (int(pwm_bytes[2*i+1]) << 8)
            if pwm_value & 0x8000:
                pwm_value = -1 * ((pwm_value ^ 0xffff) + 1)
            self.pwm[i] = pwm_value
    
    def close(self):
        self.pi.i2c_close(self.handle)
        self.pi.stop()
###
