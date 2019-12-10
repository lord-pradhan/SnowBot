import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *
import time

###---- Brain Class -----####
class Brain:

	def __init__(self):
		
		# self.checkEMButton = 0
		# self.checkReset = 0
		self.state = 'init'
		# self.snowbot.plow = 0 #over i2c
		self.mark = 0
		# self.ct1 = 0

	# def EMProc(self, snowbot):

	# 	self.snowbot.w_l = 0
	# 	self.snowbot.w_r = 0
	# 	self.snowbot.plow = 0
	# 	self.state = 'stop'

	# def resetProc(self, snowbot):

	# 	self.snowbot.w_l = 0
	# 	self.snowbot.w_r = 0
	# 	self.snowbot.plow = 0
	# 	self.state = 'init'

	def initProc(self):

		# self.arduino = Arduino(serial_input)
		self.snowbot = SnowBot(6,-6, 1.57)
		self.measurement = Measurement(self.snowbot)
		self.mark = 1
		self.path = Path(self.snowbot, self.mark)
		self.controller = Controller(self.snowbot, self.path, self.mark)
		self.sensorfusion = SensorFusion()
		self.state = 'think'
		print('go to think')

	def thinkProc(self):

		self.state = 'driveD'
		print('go to driveD')

	def driveDProc(self):

		# self.arduino.serialisation(serial_input)
		self.measurement.measurementUpdate(self.snowbot)
		self.sensorfusion.state_update(self.snowbot, self.measurement)
		self.controller.control_update(self.snowbot, self.path, self.mark)

		#check if reached boundary
		if (self.controller.in_flag == 1):
			print('reached boundary')
			self.snowbot.w_l = 0; self.snowbot.w_r = 0
			self.state = 'driveE'
			print('go to driveE')
			self.gamma_min = self.controller.gamma_prev
			self.start = self.gamma_min
			self.looped = 0
			self.snowbot.plow = 1 #write to i2c 

	def driveEProc(self):

		# self.arduino.serialisation(serial_input)
		self.measurement.measurementUpdate(self.snowbot)
		self.sensorfusion.state_update(self.snowbot, self.measurement)
		self.controller.control_update(self.snowbot, self.path, self.mark)

		# check if finished loop
		# print(self.controller.gamma_prev , self.gamma_min)

		if self.controller.gamma_prev == self.start - 1:
			self.start -= 1
			self.looped += 1

		# self.travel += np.abs(self.controller.gamma_prev - self.travel)
		if self.looped > 2:
			print('finished 1 loop')
			# check if reached innermost loop
			if self.mark >4:
				self.state = 'goHome'
				print('time to go home')
				self.snowbot.w_l = 0; self.snowbot.w_r = 0

			else:
				self.mark += 1
				self.controller.dumped_flag = 0
				print('move to inner loop')
				self.start = self.controller.gamma_prev
				self.looped = 0

		if self.controller.dumped_flag == 0 and self.looped > 1 and \
		np.isin(wrappath(self.controller.gamma_prev, self.path), Path(self.snowbot,self.mark).gamma_dump):
			
			print('move to dump')
			self.state = 'dump'
			self.snowbot.w_l = 0; self.snowbot.w_r = 0
			self.gamma_min = self.controller.gamma_prev


	def dumpProc(self):

		if self.controller.dumped_flag == 0:
			# self.arduino.serialisation(serial_input)
			self.measurement.measurementUpdate(self.snowbot)
			self.sensorfusion.state_update(self.snowbot, self.measurement)
			self.controller.control_update(self.snowbot, self.gamma_min, self.mark, mode=1)

		else:
			self.state = 'driveE'
			print('done w dump')

	def goHomeProc(self):

		if self.controller.home_flag == 0:
			# self.arduino.serialisation(serial_input)
			self.measurement.measurementUpdate(self.snowbot)
			self.sensorfusion.state_update(self.snowbot, self.measurement)
			self.controller.control_update(self.snowbot, self.gamma_min, self.mark, mode=2)

		else:
			print("reached home")
			self.state = 'done'

			












