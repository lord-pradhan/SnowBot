import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *

###---- Brain Class -----####
class Brain:

	def __init__(self):
		
		self.checkEMButton = 0
		self.checkReset = 0
		self.state = 'init'
		# self.ct1 = 0

	def EMProc(self, snowbot):

		self.snowbot.w_l = 0
		self.snowbot.w_r = 0
		self.snowbot.plow = 0
		self.state = 'stop'

	def resetProc(self, snowbot):

		self.snowbot.w_l = 0
		self.snowbot.w_r = 0
		self.snowbot.plow = 0
		self.state = 'init'

	def initProc(self, serial_input):

		self.arduino = Arduino(serial_input)
		self.measurement = Measurement(arduino)
		self.snowbot = Snowbot(measurement)
		self.boundary = Path()
		self.path = boundary.moveIn()
		self.controller = Controller(snowbot, boundary)
		self.sensorfusion = SensorFusion()
		self.state = 'think'

	def thinkProc(self, serial_input):

		self.state = 'driveD'

	def driveDProc(self, serial_input):

		self.arduino.serialisation(serial_input)
		self.measurement = measurementUpdate(self.arduino)
		self.sensorfusion.state_update(self.snowbot, self.measurement)
		self.controller.control_update(self.snowbot, self.boundary)

		#check if reached boundary
		if (self.controller.in_flag == 1):
			self.snowbot.w_l = 0; self.snowbot.w_r = 0
			self.state = 'driveE'

	def driveEProc(self, serial_input):

		self.snowbot.plow = 1

		self.arduino.serialisation(serial_input)
		self.measurement = measurementUpdate(self.arduino)
		self.sensorfusion.state_update(self.snowbot, self.measurement)
		self.controller.control_update(self.snowbot, self.boundary)

