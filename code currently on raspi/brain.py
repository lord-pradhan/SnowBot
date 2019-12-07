import numpy as np
import numpy.linalg as LA
# from usefulFuncs import *
from classDefs import *
##from DriveArduino import *
##from MPU6050 import *

###---- Brain Class -----####
class Brain:

	def __init__(self):
		
                self.checkEMButton = 0
                self.checkReset = 0
                self.state = 'init'
                # self.ct1 = 0

                self.x_pre = 0
                self.y_pre = 0
                self.theta_pre = 0
                self.x_post = 0
                self.y_post = 0
                self.theta_post = 0
                

	def initProc(self, ser, mpu):

                self.arduino = Arduino(ser)
                # self.imu = Imu()
                self.measurement = Measurement(self.arduino, mpu)
                print('before EKF', self.measurement.x, self.measurement.y, self.measurement.theta)
                self.x_pre = self.measurement.x
                self.y_pre = self.measurement.y
                self.theta_pre = self.measurement.theta

                self.snowbot = SnowBot(self.measurement)
                self.boundary = Path()
                self.controller = Controller(self.snowbot, self.boundary)
                self.sensorfusion = SensorFusion()
                print('after EKF', self.snowbot.state.x, self.snowbot.state.y, self.snowbot.state.theta)
                self.x_post = self.snowbot.state.x
                self.y_post = self.snowbot.state.y
                self.theta_post = self.snowbot.state.theta
                self.state = 'think'

	def thinkProc(self, ser, mpu):

		self.state = 'driveD'

	def driveDProc(self, ser, mpu):

                self.arduino.serialisation(ser)
                self.measurement.measurementUpdate(self.arduino, mpu)
                print('before EKF', self.measurement.x, self.measurement.y, self.measurement.theta)
                self.x_pre = self.measurement.x
                self.y_pre = self.measurement.y
                self.theta_pre = self.measurement.theta

                self.sensorfusion.state_update(self.snowbot, self.measurement)
                print('after EKF', self.snowbot.state.x, self.snowbot.state.y, self.snowbot.state.theta)

                self.x_post = self.snowbot.state.x
                self.y_post = self.snowbot.state.y
                self.theta_post = self.snowbot.state.theta


                self.controller.control_update(self.snowbot, self.boundary)

