import pigpio
import time
#import Ultrasound

class Grua(object) :
	
	def __init__ (self, pinA, pinB, pinSA, pinSB, ultraS):
	   
	    
	    self.pinA = pinA#pines de pwm de la grua
	    self.pinB = pinB
	    
	    self.pinSA = pinSA #pin que envia senal
	    self.pinSB = pinSB #pin que enciendo si el suiche se cierra
	    
	    self.ultraS = ultraS
	   
	    
	    self.pi = pigpio.pi()
	    self.pi.set_mode(pinA, pigpio.OUTPUT)
	    self.pi.set_mode(pinB, pigpio.OUTPUT)
	    
	    self.pi.set_mode(pinSA, pigpio.OUTPUT)
	    self.pi.set_mode(pinSB, pigpio.INPUT)
	    
	    self.level = 0
	    
	    self.dist = 0
	    self.swiA = 0
	    self.swiB = 0
	    
	    self.out = 200
	    	    
	def nivelA(self, level) :
		if level == 1:
			while not swiA :
				bajar()
		elif level == 2:
			while dist >= 7:
				subir()
			while dist < 7:
				bajar()
		elif level == 3:
			while dist >= 7:
				subir()
		else:
			while not swiB :
				subir()
		parar()
		
	def bajar(self):
		self.pi.set_PWM_dutycycle(self.pinA, self.out)
		self.pi.set_PWM_dutycycle(self.pinB, 0)
		
	def subir(self):
		self.pi.set_PWM_dutycycle(self.pinA, 0)
		self.pi.set_PWM_dutycycle(self.pinB, self.out)
		
	def parar(self):
		self.pi.set_PWM_dutycycle(self.pinA, 0)
		self.pi.set_PWM_dutycycle(self.pinB, 0)
		
	def subirMax(self):
		while self.pi.read(self.pinSB)==1: #Solo se activan los motores de la grua cuando el suiche de emergencia esta abierto
			self.subir()
		
		self.parar()
		
	def mover(self, dist):
		ini = self.ultraS.getDistance()
		act = self.ultraS.getDistance()
		mov = act - ini
		print(mov)
		
		while ((mov < dist-0.5) or (mov > dist-0.5)) and self.pi.read(self.pinSB)==1:
			if (mov < dist-0.5):
				self.subir()
				print(mov)
				print("subiendo")
			elif (mov > dist+0.5):
				self.bajar()
				print(mov)
				print("bajando")

			
			act = self.ultraS.getDistance()
			mov = act - ini
		
		
		self.parar()
		
	def nivel(self, n):
		act = self.ultraS.getDistance()
	
		
		if n == 0:
			self.nivel(1)
			print("llegue al nivel 1")
			self.bajar()
			time.sleep(4)
				
		elif n == 1:
			while ((act < 4) or (act > 6)):
				print(act)
				if (act < 0) or (act > 20) and self.pi.read(self.pinSB)==1:
					self.subir()
				elif (act < 4) and self.pi.read(self.pinSB)==1:
					self.subir()
				elif (act > 6) or self.pi.read(self.pinSB)==0:
					self.bajar()
				
				act = self.ultraS.getDistance()
		elif n == 2:
			while ((act < 7.5) or (act > 9)):
				print(act)
				if (act < 0) or (act > 20)  and self.pi.read(self.pinSB)==1:
					self.subir()
				elif (act < 7.5)  and self.pi.read(self.pinSB)==1:
					self.subir()
				elif (act > 9) or self.pi.read(self.pinSB)==0:
					self.bajar()
				
				act = self.ultraS.getDistance()
		
		elif n == 3:
			self.subirMax()
			
		else:
			print("El nivel indicado no existe.")
				
		self.parar()
		
	
			
		
	
		
		
	
		
		
            
				
	


