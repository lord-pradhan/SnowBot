import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *

if __name__ == '__main__':

	brain = Brain()

	while (True):
		
		if (Brain.checkEMButton == 1):
			brain.EMProc()

		else if Brain.checkReset == 1:
			brain.resetProc()

		else if brain.state == 'init':
			brain.initProc(serial_input)

		else if brain.state == 'think':
			brain.thinkProc(serial_input)

		else if brain.state == 'driveD':
			brain.driveDProc(serial_input)

		else if brain.state == 'driveE':
			brain.driveEProc(serial_input)

		else if 


