"""Aqui se encuentra la clase Magnet que nos permite controlar el electroiman."""

from time import sleep
import pigpio
  
class Magnet(object):

	def __init__(self, pin):
		
		self.pin = pin
		
		self.pi = pigpio.pi()
	
		self.pi.set_mode(self.pin, pigpio.OUTPUT)
	

	def on(self):
		"""Prende el electroiman."""
		self.pi.write(self.pin, 1)

	
	def off(self):
		"""Apaga el electroiman."""
		self.pi.write(self.pin, 0)
