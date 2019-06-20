import pigpio
import time
from subprocess import call 

class UltrasonicS(object):
	
	def __init__(self,  pinTrig, pinEcho):
		self.pinTrig = pinTrig    # pin del trigger
		self.pinEcho = pinEcho   # pin del echo
		
		self.pi = pigpio.pi()
		
		#Seteamos los pines de entrada y salida para el trigger y el echo
		
		self.pi.set_mode(self.pinTrig, pigpio.OUTPUT)
		self.pi.set_mode(self.pinEcho, pigpio.INPUT)
		
		#Nos aseguramos que el pin de salida del trigger este en Low (0)
		self.pi.write(self.pinTrig,0)
		
	def getDistance(self):
		medida = 0
		start = 0
		end = 0
		
		for i in range(0,5):
			#apagamos el pin Trig
			self.pi.write(self.pinTrig,0)
			time.sleep(2*10**-6) #esperamos dos microsegundos
			#encendemos el pin Trig
			self.pi.write(self.pinTrig,1)
			time.sleep(10*10**-6) #esperamos diez microsegundos
			#y lo volvemos a apagar
			self.pi.write(self.pinTrig,0)
		 
			#empezaremos a contar el tiempo cuando el pin Echo se encienda
			start2 = time.time()
			while self.pi.read(self.pinEcho)==0: 
				start = time.time()
				#print("marico matame")
				if -start2+start > 0.9:
					#print("no llegue perrito")
					break
				
		 
			while self.pi.read(self.pinEcho)==1:
				end = time.time()
		 
			#La duracion del pulso del pin Echo sera la diferencia entre
			#el tiempo de inicio y el final
			duracion = end-start
		 
			#Este tiempo viene dado en segundos. Si lo pasamos
			#a microsegundos, podemos aplicar directamente las formulas
			#de la documentacion
			duracion = duracion*10**6
			#hay que dividir por la constante que pone en la documentacion, nos dara la distancia en cm
			medida += duracion/58 
			time.sleep(0.1)
		
		medida = medida/5
		
		return medida


	
	
