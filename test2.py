from MotorControlBeta import Motor
import pigpio
from time import sleep

motorL = Motor("Motor Izquierdo", 18, 17, 12, 6)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)


#Para un tiempo T sin PID

t = 0
dt = 0.1

while t < 30:
	#motorR.run(50)
	#motorL.run(50)
	
	motorR.encoder()
	motorL.encoder()

	print( str(motorL.getTicks()) + " " + str(motorR.getTicks()) )

	t = t + dt
	sleep(dt)

motorR.stop()
motorL.stop()
"""
#Para d Ticks de encoder con PID
motorR.clearTicks()
motorL.clearTicks()
d = 12

while motorL.getTicks() < d and motorR.getTicks() < d :
"""
	
