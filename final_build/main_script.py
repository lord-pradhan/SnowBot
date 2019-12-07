import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *

if __name__ == '__main__':

	brain = Brain()

	while (True):
		
		if (Brain.checkEMButton == 1):
			brain.EMProc()

		elif Brain.checkReset == 1:
			brain.resetProc()

		elif brain.state == 'init':
			brain.initProc(serial_input)

		elif brain.state == 'think':
			brain.thinkProc(serial_input)

		elif brain.state == 'driveD':
			brain.driveDProc(serial_input)

		elif brain.state == 'driveE':
			brain.driveEProc(serial_input)

		elif brain.state == 'dump':
			brain.dumpProc(serial_input)

		elif brain.state == 'goHome':
			brain.goHomeProc(serial_input)

		elif brain.state == 'done':
			pass
