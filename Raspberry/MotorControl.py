import pigpio
import time

class Motors(object):
	
	def __init__(self, pinL, pinR):
		""" Recibe pinL (Pin del motor izquierdo) y pinR 
		(Pin del motor derecho)"""
		self.pinL = pinL
		self.pinR = pinR
		self.pi = pigpio.pi()

		self.pi.set_mode(self.pinL, pigpio.OUTPUT)
		self.pi.set_mode(self.pinR, pigpio.OUTPUT)
        
        
	def run(self, power):
		"""
		Power va entre -100 y 100 y hace un mapeo para transformar los 
		valores a las frecuenciasa qu recibe la funicion servo_pulsewidth
		que van de 1000 a 2000, 1500 para parar <1500 hacia atras, y
		>1500 para adelante cada motor
		"""
		if power >= 100:
			power = 100
		elif power <=-100:
			power = -100
		self.pi.set_servo_pulsewidth(self.pinL, -5*power+1500)
		self.pi.set_servo_pulsewidth(self.pinR, 5*power+1500)
      
      
	def stop(self):
		self.pi.set_servo_pulsewidth(self.pinL, 1500)
		self.pi.set_servo_pulsewidth(self.pinR, 1500)


	def setMotorL(self, power):
		if power >= 100:
			power = 100
		elif power <=-100:
			power = -100
		self.pi.set_servo_pulsewidth(self.pinR, -5*power+1500) # Estan al revez, pero, funciona
		
		
	def setMotorR(self, power):
		if power >= 100:
			power = 100
		elif power <=-100:
			power = -100
		self.pi.set_servo_pulsewidth(self.pinL, 5*power+1500)
