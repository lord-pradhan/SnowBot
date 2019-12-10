import serial
import math
from usefulFuncs import *
import numpy as np
import time

###
class IMUArduino:

    def __init__(self):
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate = 115200)
##        self.bias =[-19.575, 16.65]
##        self.scale = [0.963556851, 1.039308176]
        self.bias = [-10.875, 15.525]
        self.scale = [0.932461874, 1.078085642]

        # N, W, S_H, S_L, E
##        self.li_cal = [15.79, 113.76, 180.0, -180.0, -105.81]
##        self.li_cal = [17.91831055, 110.7120346, 170.6936832, -116.9028195]
        self.li_cal = [-2.226950638, 114.7529102, 172.9929527, -121.182844]
        self.li_act = [0, 90, 180, -180, -90]
        # N-W, W-SH, SL-E, E-N
##        self.li_slopes = [0.918644553, 1.35875955, 1.213143705, 0.740099514]
##        self.li_slopes = [0.969893179, 1.500458924, 1.243033877, 0.769870226]
        self.li_slopes = [0.769363199, 1.545328542, 1.367278227, 0.742679385]
##        self.li_slopes = self.__calcSlopes()
        
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.raw_yaw = 0.0

        self.Noffset = wrap2pi(210*np.pi/180)

        self.update()
        self.yaw_fin = wrap2pi(self.magAngle)
        self.time = time.time()

    def __calcSlopes(self):
        slopes = [0.0] * (len(self.lininterp_cal) - 1)
        cInd = 0;
        for sInd in range(len(slopes)-1):
            if(self.lininterp_cal[cInd] > self.lininterp_cal[cInd+1]):
                cInd += 1
            slopes[sInd] = (self.li_act[cInd+1] - self.li_act[cInd]) / (self.li_cal[cInd+1] - self.li_cal[cInd])
            cInd += 1
        slopes[sInd] = (self.li_act[-1] - self.li_act[0]) / (self.li_cal[-1] - self.li_cal[0])

    def update(self):
        while(1):
            self.ser.reset_input_buffer()
            while(1):
                try:
                    inData = self.ser.readline().decode('ascii')
                    break
                except UnicodeDecodeError:
                    print('IMU data not parsable.')
            inData = inData.replace('\r\n',"")
            data = inData.split(',')
            
            try:
                self.gyro = [float(data[0]), float(data[1]), float(data[2])]
                self.mag = [float(data[3]), float(data[4]), float(data[5])]
                break
            except:
                print('Inconsistent data from IMU.')
        mag_x_corr = (self.mag[0] - self.bias[0]) * self.scale[0]
        mag_y_corr = (self.mag[1] - self.bias[1]) * self.scale[0]
        
        angle_before = math.degrees(math.atan2(mag_y_corr, mag_x_corr))
        self.raw_yaw = angle_before
        if(self.li_cal[0] <= angle_before < self.li_cal[1]):
            angle_after = self.li_slopes[0] * (angle_before - self.li_cal[0]) + self.li_act[0]
        elif(self.li_cal[1] <= angle_before < self.li_cal[2]):
            angle_after = self.li_slopes[1] * (angle_before - self.li_cal[1]) + self.li_act[1]
        elif(self.li_cal[2] <= angle_before < 180.0):
            angle_after = self.li_act[3] + self.li_slopes[2] * (angle_before - self.li_cal[2])
        elif(-180.0 <= angle_before < self.li_cal[3]):
            angle_after = self.li_act[3] + self.li_slopes[2]*(self.li_act[2]-self.li_cal[2]) + self.li_slopes[2]*(angle_before+180)
        elif(self.li_cal[3] <= angle_before < self.li_cal[0]):
            angle_after = self.li_slopes[3] * (angle_before - self.li_cal[3]) + self.li_act[4]
        print(angle_after)
        self.magAngle = angle_after*np.pi/180.0
        self.normMagAngle = self.magAngle + self.Noffset
    
    def getYaw(self):
        self.update()
        

        ## complementary filter
        yaw_prev = wrap2pi(self.yaw_fin)
        gyro_norm = (self.gyro[2]+1.57)*np.pi/180.0
        currTime = time.time()
        self.yaw_fin = wrap2pi( 0.20*wrap2pi(yaw_prev + gyro_norm*(currTime - self.time) ) + 0.80*wrap2pi(self.magAngle) )
        self.time = currTime

        return wrap2pi(self.Noffset + self.yaw_fin)

###

if __name__ == '__main__':
    imu = IMUArduino()

##    while(1):
##        print(imu.getYaw())
##        imu.getYaw()
##        imu.update()
##        print(imu.mag[0], ',', imu.mag[1])

    
    avg_sum = 0
    cnt = 1
    while(1):
        imu.update()
        avg_sum += imu.gyro[2]
        print(avg_sum / cnt)
        cnt += 1
        
