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
		self.posPass = 0
	
	def getPos(self):
		#Obtinene la posicion del encoder en cierto momento
		DutyCycleMax= 0.971
		DutyCycleMin = 0.029
	
		x = 0.2
		
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
			#out = (DutyCycle-DutyCycleMin)*360/(DutyCycleMax-DutyCycleMin+1) 
			out = abs((DutyCycle * 360 - 30 )*360/300)
			#Se envia la senal si solo si el tiempo del cyclo completo esta en el el rango apropiado, si no se desecha y se hace otra lectura
			 
			if tCycle > 0.001 and tCycle < 0.0012:
				self.posPass = out * x + (1-x)* self.posPass
				return self.posPass
			else: 
				pass

	def getDif(self):	
		#Obtiene la diferencia de grados del encoder

		prev = self.getPos()
		now  = self.getPos()	
		
		#El analisis de casos se realiza para saber si ocurrio un giro del 4to al 1er cuadrante o viceversa y se hacen los calculos necesarios paraajustar la diferencia
		if now > 270 and prev < 90:
			dif = now - (prev + 360)
		elif  prev > 270 and now < 90: 
			dif = (now + 360) - prev 
		else:
			dif = now - prev 
			
		return dif

	def getPosPlus(self):
		#Obtinene la posicion del encoder en cierto momento y el tiempo que tardo en hacer la lectura
		DutyCycleMax= 97.1
		DutyCycleMin = 2.9
		x = 0.9
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
			DutyCycle =100*(tHigh / tCycle)
			out = (DutyCycle-DutyCycleMin)*360/(DutyCycleMax-DutyCycleMin+1) #Se supone que deberia llevar esto pero meh
			
			#Se envia la senal si solo si el tiempo del cyclo completo esta en el el rango apropiado, si no se desecha y se hace otra lectura
			if tCycle > 0.001 and tCycle < 0.0012:
				self.posPass = out * x + (1-x)* self.posPass
				return [self.posPass, tCycle]
			else: 
				pass

	def getAngularVelocity(self):
		#Obtiene la velocidad angular del motor
		
		pi = 3.141592654 #Valor de pi
		
		[prev,tOld] = self.getPosPlus()
		[now, tNow]= self.getPosPlus()	
	
		#El analisis de casos se realiza para saber si ocurrio un giro del 4to al 1er cuadrante o viceversa y se hacen los calculos necesarios paraajustar la diferencia
		if now > 270 and prev < 90:
			dif = now - (prev + 360)
		elif  prev > 270 and now < 90: 
			dif = (now + 360) - prev 
		else:
			dif = (now - prev)
		#Ecuacion de la velocidad angular
		
		w = pi*dif/((tOld+tNow) * 180 )
			
		return w
		
