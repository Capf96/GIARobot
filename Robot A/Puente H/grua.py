# Faltan los metodos de actualizar los botones
import pigpio
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)
s = []

class Grua(object) :
	
	def __init__ (self, pinA, pinB):
		self.pinA = pinA
		self.pinB = pinB
	    
		self.pi = pigpio.pi()
		self.pi.set_mode(pinA, pigpio.OUTPUT)
		self.pi.set_mode(pinB, pigpio.OUTPUT)
	    
		self.dist = 0
		self.swiA = true   # Boton inferior
		self.swiB = true   # Boton superior
		
		self.level = 0
		self.out = 200     # Voltaje que se le pasa al motor
	    	    
	def nivel(self, level) :
		actualizar_ultrasonido()
		if level == 1:
			while not swiA :
				bajar()
		elif level == 2:
			while dist >= 7:
				subir()
				actualizar_ultrasonido()
			while dist < 7:
				bajar()
				actualizar_ultrasonido()
		elif level == 3:
			while dist >= 7:
				subir()
				actualizar_ultrasonido()
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
		
	def actualizar_ultrasonido(self):
		data = ''.join(list(ser.readline())[:-2])
	    	if data:
			result = str(ser.readline().strip())
			s = result.decode('utf-8').split(',')
			dist = s[len(s)-2] 
		
            
				
	


