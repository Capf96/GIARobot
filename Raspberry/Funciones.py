import pigpio
import serial

from MotorControl import Motor
from time import sleep, time
from electroIman import Magnet
from Sensores import QTR, Sensor
from UltraSonic import UltrasonicS
from subprocess import call 
from multiprocessing import Process
ser = serial.Serial('/dev/ttyUSB0', 57600)
# Encoder provides a resolution of 3600 ticks per revolution.
# Un tick es la division minima del giro del moto que proporciona el encoder
ser.flushInput()


class Robot(object):	
				
	def getNearBlock(self, pwm):
		"""
		El robot se acercara al bloque hasta estar una distancia menor a limit del mismo.
		"""	
		limit = 5.5 #cm
				
		while self.ultSndR.getDistance() > limit:
			self.forward(1, pwm)

		self.stop()
		

	

		
			
				
	
	
