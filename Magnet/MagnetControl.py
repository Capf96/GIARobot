import pigpio

class Magnet(object):
	"""docstring for Magnet"""
	def __init__(self, pin):

		self.pin = pin
		self.pi = pigpio.pi()

		self.pi.set_mode(self.pin, pigpio.OUTPUT)
	
	def turn(self, on):
		if on == True:
			self.pi.write(self.pin, 1)

		elif on == False:
			self.pi.write(self.pin, 0)

