import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *
import time

###---- Brain Class -----####
class Brain:

	def __init__(self, snowbot):
		
		self.checkEMButton = 0
		self.checkReset = 0
		self.state = 'init'
		self.snowbot.plow = 0 #over i2c
		self.mark = 0
		# self.ct1 = 0

        self.x_pre = 0
        self.y_pre = 0
        self.theta_pre = 0
        self.x_post = 0
        self.y_post = 0
        self.theta_post = 0


	def initProc(self, ser, mpu):

		# self.arduino = Arduino(serial_input)
		# self.measurement = Measurement(arduino)
		# self.snowbot = Snowbot(measurement)
		
		# self.path = Path(self.snowbot, self.mark)
		# self.controller = Controller(self.snowbot, self.path, self.mark)
		# self.sensorfusion = SensorFusion()
		# self.state = 'think'

		self.arduino = Arduino(ser)
		self.mark = 1

        self.measurement = Measurement(self.arduino, mpu)

        print('before EKF', self.measurement.x, self.measurement.y, self.measurement.theta)
        self.x_pre = self.measurement.x
        self.y_pre = self.measurement.y
        self.theta_pre = self.measurement.theta

        self.snowbot = SnowBot(self.measurement)
        self.path = Path(self.snowbot, self.mark)
        self.controller = Controller(self.snowbot, self.path, self.mark)
        self.sensorfusion = SensorFusion()

        print('after EKF', self.snowbot.state.x, self.snowbot.state.y, self.snowbot.state.theta)
        self.x_post = self.snowbot.state.x
        self.y_post = self.snowbot.state.y
        self.theta_post = self.snowbot.state.theta
        self.state = 'think'


	def thinkProc(self, ser, mpu):

		self.state = 'driveD'

	def driveDProc(self, serial_input):

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

		self.controller.control_update(self.snowbot, self.path, self.mark)

		#check if reached boundary
		if (self.controller.in_flag == 1):
			self.snowbot.w_l = 0; self.snowbot.w_r = 0
			self.state = 'driveE'
			self.gamma_min = self.controller.gamma_prev
			self.snowbot.plow = 1 #write to i2c

	def driveEProc(self, serial_input):

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

		self.controller.control_update(self.snowbot, self.path, self.mark)

		# check if finished loop
		if self.controller.gamma_prev - self.gamma_min > 2*len(self.path.x_d):

			# check if reached innermost loop
			if self.mark >3:
				self.state = 'goHome'
				self.snowbot.w_l = 0; self.snowbot.w_r = 0

			else:
				self.mark -= 1

		# check if reached dump location
		if self.controller.gamma_prev - self.gamma_min > len(self.path.x_d) \
		and np.isin(wrapPath(self.controller.gamma_prev), Path(self.snowbot,self.mark).gamma_dump):
			
			self.state = 'dump'
			self.snowbot.w_l = 0; self.snowbot.w_r = 0
			self.gamma_min = self.controller.gamma_prev


	def dumpProc(self, serial_input):

		if self.controller.dumped_flag == 0:
			self.arduino.serialisation(serial_input)
			self.measurement = measurementUpdate(self.arduino)
			self.sensorfusion.state_update(self.snowbot, self.measurement)
			self.controller.control_update(self.snowbot, self.gamma_min, self.mark, mode=1)

		else:
			self.state = 'driveE'

	def goHomeProc(self, serial_input):

		if self.controller.home_flag == 0:
			self.arduino.serialisation(serial_input)
			self.measurement = measurementUpdate(self.arduino)
			self.sensorfusion.state_update(self.snowbot, self.measurement)
			self.controller.control_update(self.snowbot, self.gamma_min, self.mark, mode=2)

		else:
			print("reached home")
			self.state = 'done'

			












