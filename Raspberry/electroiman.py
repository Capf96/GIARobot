from time import sleep
import pigpio
  
class Magnet(object):
	def __init__(self, pin):
		
		self.pin = pin
		
		self.pi = pigpio.pi()
	
		self.pi.set_mode(self.pin, pigpio.OUTPUT)
	
	def on(self):
		self.pi.write(self.pin, 1)
	
	def off(self):
		self.pi.write(self.pin, 0)
