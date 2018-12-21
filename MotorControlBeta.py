#!/usr/bin/python

import pigpio

class Motor(object):
    def __init__(self, name, pinN, pinM, pinB, pinA):
        self.name = name
        self.pinN = pinN
        self.pinM = pinM
        self.pinB = pinB
        self.pinA = pinA
        
        self.pi = pigpio.pi()
        self.clkLastState = 0
        self.encoderPos = 0

    def setup(self):
        self.pi.set_mode(self.pinN, pigpio.OUTPUT)
        self.pi.set_mode(self.pinM, pigpio.OUTPUT)

        self.pi.set_mode(self.pinB, pigpio.INPUT)
        self.pi.set_mode(self.pinA, pigpio.INPUT)
        self.clkLastState = self.pi.read(self.pinA)

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

    def encoder(self):
        
        self.clkState = self.pi.read(self.pinA)
        self.dtState = self.pi.read(self.pinB)
        
        if self.clkState != self.clkLastState:
            
            if self.dtState != self.clkState:
                
                self.encoderPos +=1
            
            else:
                self.encoderPos -=1
		
		self.clkLastState = self.clkState
		
    
    def getTicks(self):
        return  self.encoderPos

    def clearTicks(self):
        self.encoderPos = 0
        

        


        
