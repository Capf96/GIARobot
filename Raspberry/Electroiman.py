"""Aqui se encuentra la clase Magnet que nos permite controlar el electroiman."""

from time import sleep
import pigpio
  
class Magnet(object):

	def __init__(self, pinA, pinB):
		
		self.pinA = pinA
		self.pinB = pinB
		self.pi = pigpio.pi()
	
		self.pi.set_mode(self.pinA, pigpio.OUTPUT)
		self.pi.set_mode(self.pinB, pigpio.OUTPUT)
	

	def on(self):
		"""Prende el electroiman."""
		self.pi.write(self.pinA, 1)
		self.pi.write(self.pinB, 0)

	
	def off(self):
		"""Apaga el electroiman."""
		self.pi.write(self.pinA, 0)
		self.pi.write(self.pinB, 0)
