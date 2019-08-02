import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pines del Puente h
# Par 1
pinY = 24
pinW = 23
# Par 2
pinB = 22
pinR = 27


# Set pin states
GPIO.setup(pinY, GPIO.OUT)
GPIO.setup(pinW, GPIO.OUT)
GPIO.setup(pinB, GPIO.OUT)
GPIO.setup(pinR, GPIO.OUT)


# Function for step sequence
def setStep(w1, w2, w3, w4):
  GPIO.output(pinY, w1)
  GPIO.output(pinW, w2)
  GPIO.output(pinB, w3)
  GPIO.output(pinR, w4)


def stepper(steps, delay):
	"""Hace girar el Stepper Motor "steps" pasos con un retraso de "delay" segundos entre cada paso. 
		El sentido del giro esta determinado por el signo de la variable "steps". Se recomienda colocar el delay en 0.0055"""
		
	if steps > 0:
		for i in range(0, steps):
			setStep(1,0,1,0)
			time.sleep(delay)
			setStep(0,1,1,0)
			time.sleep(delay)
			setStep(0,1,0,1)
			time.sleep(delay)
			setStep(1,0,0,1)
			time.sleep(delay)
			
	else:
		for i in range(0, -steps):
			setStep(1,0,0,1)
			time.sleep(delay)
			setStep(0,1,0,1)
			time.sleep(delay)
			setStep(0,1,1,0)
			time.sleep(delay)
			setStep(1,0,1,0)
			time.sleep(delay)

stepper(200, 0.0055)
