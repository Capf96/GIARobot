from MotorControl import Motor
import pigpio
from time import sleep

motorL = Motor("Motor Izquierdo", 18, 17, 6, 12)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)


#Para un tiempo T sin PID

t = 0
dt = 0.1

while t < 30000:
	
	
	

	print( str(motorL.encoder.getTicks()) + " " + str(motorR.encoder.getTicks()) )

	t = t + dt
	sleep(dt)
motorL.stop()
motorR.stop()
