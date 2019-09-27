import pigpio
from time import time 

	
class Encoder(object):
	def __init__(self, pin):
		"""
		Clase para utilizar los encoders de los servos Parallax directamente en la Raspberry 
		"""
		
		self.pin = pin #Numero del pin gpio al que esta conectado el encoder
		self.pi = pigpio.pi()
		self.pi.set_mode(self.pin, pigpio.INPUT)
	
	def getPos(self):
		DutyCycleMax= 0.917
		DutyCycleMin = 0.029
		while True:
			to = time() #Tomamos un tiempo to
			#Medimos el tiempo en el que el la senal del encoder es high(1)
			while self.pi.read(self.pin)==1: 
				pass
			tHigh = time()-to
			#Medimos el tiempo de la resto de la senal completa ("el ciclo entero (high+low)")
			while self.pi.read(self.pin)==0:
				pass
			tCycle = time()-to
			
			#Aplicamos la formula que proporciona el datasheet
			DutyCycle =(tHigh / tCycle)
			out = (DutyCycle-DutyCycleMin)*360#/(DutyCycleMax-DutyCycleMin+1) Se supone que deberia llevar esto pero meh
			
			#Se envia la senal si solo si el tiempo del cyclo completo esta en el el rango apropiado, si no se desecha y se hace otra lectura
			if tCycle > 0.001 and tCycle < 0.0012:
				return out
			else: 
				pass


