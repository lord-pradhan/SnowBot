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

	# def EMProc(self):

	# 	nullstate = 

	def initProc(self, serial_input):

		self.arduino = Arduino(serial_input)
		self.measurement = Measurement(self.arduino)
		self.snowbot = Snowbot(self.measurement)
		self.boundary = Path()
		self.controller = Controller(self.snowbot, self.boundary)
		self.sensorfusion = SensorFusion()
		self.state = 'think'

	def thinkProc(self, serial_input):

		self.state = 'driveD'

	def driveDProc(self, serial_input):

		self.arduino.serialisation(serial_input)
		self.measurement.measurementUpdate(self.arduino)
		self.sensorfusion.state_update(self.snowbot, self.measurement)
		self.controller.control_update(self.snowbot, self.boundary)