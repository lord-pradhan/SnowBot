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

	def initProc(self):

		arduino = Arduino(serial_input)
		measurement = Measurement(arduino)
		snowbot = Snowbot(measurement)
		boundary = Path()
		controller = Controller(snowbot, boundary)
		sensorfusion = SensorFusion()
		self.state = 'think'

	def thinkProc(self):

		self.state = 'driveD'

	def driveDProc(self):

		arduino.serialisation()
		measurement = measurementUpdate(arduino)
		sensorfusion.state_update(snowbot, measurement)
		controller.control_update(snowbot, boundary)
		
