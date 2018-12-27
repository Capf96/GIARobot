from MotorControl import Motor
import pigpio
from time import sleep

motorL = Motor("Motor Izquierdo", 18, 17, 6, 12)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)

kp = 0.75
ki = 0
kd = 0.0025
dt = 0.01
pdif = 0
while motorL.encoder.getTicks() < 0 and  motorR.encoder.getTicks() < 0:
	
	ticksLeft = motorL.encoder.getTicks()
	ticksRight = motorR.encoder.getTicks()
	
	dif= ticksRight - ticksLeft
	
	delta = dif * kp + (dif - pdif / dt) *kd

	motorL.run(50 + delta) # 
	
	motorR.run(50) 
	
	print( str(motorL.encoder.getTicks()) + " " + str(motorR.encoder.getTicks()) )
	
	pdif= dif
	
	sleep(dt)

motorL.stop()
motorR.stop()
