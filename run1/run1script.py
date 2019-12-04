import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *
from brain import *

if __name__ == '__main__':

	brain = Brain()
	# brain.state = 'init'

	while (True):

		# if (brain.checkEMButton == 1):
		# 	brain.EMProc()

		# else if brain.checkReset == 1:
		# 	brain.resetProc()

		if brain.state == 'init':
			brain.initProc(serial_input)

		else if brain.state == 'think':
			brain.thinkProc(serial_input)

		else if brain.state == 'driveD':
			brain.driveDProc(serial_input)