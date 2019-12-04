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
			brain.nullProc()

