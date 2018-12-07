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
        self.encoder =  Encoder(self.pi, pinA, pinB)

    def setup(self):
        self.pi.set_mode(self.pinN, pigpio.OUTPUT)
        self.pi.set_mode(self.pinM, pigpio.OUTPUT)
        

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

    def readEncoder(self):
        return self.encoder.getTicks()


        
