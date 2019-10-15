from ArduinoReader import Arduino

import pigpio
import time


arduino = Arduino()
class Gyroscope(object) :
	
	def __init__ (self, pAx, pAy pAz, pGx, pGy, pGz):
		
		self.pAx = pAx
		self.pAy = pAy
		self.pAz = pAz
		
		self.pGx = pGx
		self.pGy = pGy
		self.pGz = pGz
		
	
	def getValues(self):
		
		return 
		
