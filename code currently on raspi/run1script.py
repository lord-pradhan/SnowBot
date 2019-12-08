import numpy as np
import numpy.linalg as LA
# from usefulFuncs import *
##from classDefs import *
from brain import *
from DriveArduino import *

from IMUArduino import *
import serial
import time

if __name__ == '__main__':

        brain = Brain()
        ##	theta_init = 90*np.pi/180.0
        brain.state = 'init'
        ser = serial.Serial('/dev/ttyS0', baudrate = 115200, timeout=1) # COM port may need to be changed - REMEMBER

        ##	mpu = MPU6050(theta_init)

        mpu = IMUArduino()
        drivearduino = DriveArduino(8)
        start = time.time()

        while (True):

                # if (brain.checkEMButton == 1):
                # 	brain.EMProc()

                # else if brain.checkReset == 1:
                # 	brain.resetProc()

                if brain.state == 'init':
                        brain.initProc(ser, mpu)
                        print('init done')

                elif brain.state == 'think':
                        brain.thinkProc(ser, mpu)
                        print('think done')

                elif brain.state == 'driveD':
                        time.sleep(0.3)
                        brain.driveDProc(ser,mpu, drivearduino)
                        print('drive done')

                # send control input back to system
                print('control inputs LR', brain.snowbot.w_l, brain.snowbot.w_r)
                rpm_arr = np.array([brain.snowbot.w_r, brain.snowbot.w_r, brain.snowbot.w_l, brain.snowbot.w_l])
                drivearduino.set_rpm(30*rpm_arr/np.pi)



                ##		brain.controller.x_track, brain.controller.y_track
                ##		brain.x_pre, brain.y_pre, brain.theta_pre
                ##		brain.x_post, brain.y_post, brain.theta_post

                if time.time() - start > 0.1:
                        
                        str_send = '_BOTPOS,' + str(round(brain.controller.x_track,2)) + ',' + str(round(brain.controller.y_track,2)) + ',' + \
                                   str(round(brain.x_pre,2)) + ',' + str(round(brain.y_pre,2)) + ',' + str(round(brain.theta_pre,3)) + ',' +\
                                   str(round(brain.x_post,2)) + ',' + str(round(brain.y_post,2)) + ',' + str(round(brain.theta_post,3))+ '\n';

                        
##                        str_send = '_BOTPOS,' + str(round(brain.controller.x_track,2)) + ',' + str(round(brain.controller.y_track,2)) + ',' + \
##                        str(round(brain.x_pre,2)) + ',' + str(round(brain.y_pre,2)) + ',' + str(round(brain.theta_pre,3)) + ',' +\
##                        str(0) + ',' + str(0) + ',' + str(round(wrap2pi(mpu.normMagAngle),3))+ '\n';

                        ser.write(bytes(str_send.encode('ascii')))
                        start = time.time()

