import pigpio
import time

class Switch(object) :
	
	def __init__ (self, pinSA, pinSB):
		self.pi = pigpio.pi()
		
		self.pinSA = pinSA #pin que envia senal
		self.pinSB = pinSB #pin que recibe senal
		
		#Se inicializan los pines del suiche
		
		self.pi.set_mode(self.pinSA, pigpio.OUTPUT)
		self.pi.set_mode(self.pinSB, pigpio.INPUT)
		
		self.pi.write(self.pinSA,1)
	  
		
	
	def getVal(self):
		return self.pi.read(self.pinSB)
		
			



"""
suiche = Switch(9,10)	   
while True:
	print(suiche.pi.read(suiche.pinSB))		

"""	
		
		
	
		
		
            
				
	

