#!/usr/bin/python
import pigpio
import time
from RotaryEncoder import Encoder
 

class Motor(object):
    def __init__(self, name, pinN, pinM, pinB, pinA):
        self.name = name
        self.pinN = pinN
        self.pinM = pinM
        self.pinB = pinB
        self.pinA = pinA
        self.pi = pigpio.pi()
       
        self.encoder = Encoder(self.pi, self.pinB, self.pinA)

        self.pi.set_mode(self.pinN, pigpio.OUTPUT)
        self.pi.set_mode(self.pinM, pigpio.OUTPUT)
       
        self.nTick = 0
        self.pTick = 0
        
    def run(self, power):
        if power >= 100:
            power = 100
        elif power <=-100:
            power = -100
        out = (power * 177.5)/ 100
        if out >= 0:
            self.pi.set_PWM_dutycycle(self.pinM, out)
            self.pi.set_PWM_dutycycle(self.pinN, 0)
        elif out < 0:
            out = -out
            self.pi.set_PWM_dutycycle(self.pinM, 0)
            self.pi.set_PWM_dutycycle(self.pinN, out)
      
    def stop(self):
        self.pi.set_PWM_dutycycle(self.pinM, 0)
        self.pi.set_PWM_dutycycle(self.pinN, 0)

    def getTicks(self):
        return self.encoder.getTicks()
        
    def turnTicks(self, ticks, speed):
		self.ntick = self.getTicks()
		dif = self.nTick - self.pTick
		
		
		if ticks > 0:
			
			if speed < 0 :
				speed = -speed
				
			self.pTick = self.getTicks()
			while dif < ticks:
				dif = self.nTick - self.pTick
				self.run(speed)
				self.nTick = self.getTicks()
				#print(str(dif))
			
		elif ticks < 0:
			
			if speed > 0:
				speed = -speed
			
			self.pTick = self.getTicks()
			while dif > ticks:
				dif = self.nTick - self.pTick
				self.run(speed)
				self.nTick = self.getTicks()
				#print(str(dif))
		
		self.stop()
        

        


        
