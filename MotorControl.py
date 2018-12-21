#!/usr/bin/python
from RPi import GPIO
import time

class Motor(object):
    def __init__(self, name, pinN, pinM, pinB, pinA):
        self.name = name
        self.pinN = pinN
        self.pinM = pinM
        self.pinB = pinB
        self.pinA = pinA
        self.GPIO = GPIO
        encoderPos = 0

    def setup(self):
        #self.pi.set_mode(self.pinN, pigpio.OUTPUT)
        #self.pi.set_mode(self.pinM, pigpio.OUTPUT)
        #GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pinN, GPIO.OUT)
        GPIO.setup(self.pinM, GPIO.OUT)
        
        GPIO.setup(self.pinA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pinB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


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

    def Encoder(self):
        n = 0
        encoderPinLast = n
        while True:
            encoderPinLast = n
            n = GPIO.input(self.pinB)
            if( encoderPinLast == 0 and n==1):
                if(GPIO.input(self.PinA) == 0):
                    encoderPos +=1
                else:
                    encoderPos -=1

    def GetTicks(self):
        return encoderPos
        

        


        
