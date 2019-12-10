import numpy as np
import numpy.linalg as LA
from usefulFuncs import *
from classDefs import *

if __name__ == '__main__':

	brain = Brain()

	while ():
		
		if (Brain.checkEMButton == 1):
			brain.EMProc()

		elif Brain.checkReset == 1:
			brain.resetProc()

		elif brain.state == 'init':
			brain.initProc()

		elif brain.state == 'think':
			brain.thinkProc()

		elif brain.state == 'driveD':
			brain.driveDProc()

		elif brain.state == 'driveE':
			brain.driveEProc()

		elif brain.state == 'dump':
			brain.dumpProc()

		elif brain.state == 'goHome':
			brain.goHomeProc()

		elif brain.state == 'done':
			pass

