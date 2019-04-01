from MotorControl import Motor
import pigpio
from time import sleep

motorL = Motor("Motor Izquierdo", 18, 17, 26, 20)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)

kp = 0.75
ki = 0
kd = 0.0025
dt = 0.01
pdif = 0
while motorL.encoder.getTicks() < 90000 and  motorR.encoder.getTicks() < 90000:
	
	ticksLeft = motorL.encoder.getTicks()
	ticksRight = motorR.encoder.getTicks()
	
	
	
	print( str(motorL.encoder.getTicks()) + " " + str(motorR.encoder.getTicks()) )
	
	
	
	sleep(dt)

motorL.stop()
motorR.stop()

