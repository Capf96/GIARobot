import pigpio

class Grua(object) :
	
	def __init__ (self, pinA, pinB):
	   
	    
	    self.pinA = pinA
	    self.pinB = pinB
	    
	    self.pi = pigpio.pi()
	    self.pi.set_mode(pinA, pigpio.OUTPUT)
	    self.pi.set_mode(pinB, pigpio.OUTPUT)
	    self.level = 0
	    
	    self.dist = 0
	    self.swiA = 0
	    self.swiB = 0
	    
	    self.out = 200
	    	    
	def nivel(self, level) :
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
		
            
				
	


