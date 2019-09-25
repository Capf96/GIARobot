import pigpio
import time

class Camara(object):
	
	def __init__(self, pin):
		""" Recibe del pin del servo que mueve a la camara"""
		self.pin = pin
		self.pi = pigpio.pi()

		self.pi.set_mode(self.pin, pigpio.OUTPUT)
        
        
	def turn(self, power, frente):
		"""
		Gira la camra un con una potencia de power, la variable booleana
		frente define el sentido en que va a girar
		"""
		if frente
			power *= -1
			
		if power >= 100:
			power = 100
		elif power <=-100:
			power = -100
		
		self.pi.set_servo_pulsewidth(self.pin, -5*power+1500)
		sleep(2)
		power = 0
		self.pi.set_servo_pulsewidth(self.pin, -5*power+1500)

