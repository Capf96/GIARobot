from MotorControl import Motor
import pigpio
from time import sleep

motorL = Motor("Motor Izquierdo", 18, 17, 6, 12)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)

velocidad = 50
kp = 0.0025
ki = 0
kd = 0.0025

d = 10000
dt = 0.05

# Avanza en linea recta una distancia d = 10 000
while motorL.encoder.getTicks() < d and  motorR.encoder.getTicks() < d :
	
	ticksLeft = motorL.encoder.getTicks()
	ticksRight = motorR.encoder.getTicks()
	
	dif= ticksRight - ticksLeft
	der = dif - difpast
	
	salidapwm = dif * kp +  der * kd
	
	ifsalidapwm < 0:
		motorL.run(velocidad + salidapwm) 
		motorR.run(velocidad) 
	if salidapwm > 0:
		motorL.run(velocidad + salidapwm) 
		motorR.run(velocidad) 
	
	print( str(motorL.encoder.getTicks()) + " " + str(motorR.encoder.getTicks()) )
	
	difpast = dif
	
	sleep(dt)

motorL.stop()
motorR.stop()
