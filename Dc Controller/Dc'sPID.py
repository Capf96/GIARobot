from time import sleep
import pigpio
from RPi import GPIO
from RotaryEncoder import Encoder



pi = pigpio.pi()
EncoderRight= Encoder(pi, 6, 12)
EncoderLeft= Encoder(pi, 19, 16)


pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)



def PID(Speed, Movement, tiempo_final):

	dt = 0.05
	Kp = 0.4
	Kd = 0

	InitialLeft = EncoderLeft.getTicks() 
	InitialRight = EncoderRight.getTicks() 

	if Movement=="Foward":
		SpeedLeft = Speed
		SpeedRight = Speed
		Kp = Kp
		Kd = Kd
	elif Movement == "TurnRight":
		SpeedLeft  = Speed
		SpeedRight = -Speed
		Kp = Kp
		Kd = Kd
	elif Movement == "TurnLeft":
		SpeedLeft  = -Speed
		SpeedRight = Speed
		Kp = Kp
		Kd = Kd
	elif Movement == "Backwards":
		SpeedLeft = -Speed
		SpeedRight = -Speed
		Kp = Kp
		Kd = Kd
		

	tiempo=0
	ActualRight = 0
	ActualLeft = 0
	while(tiempo < tiempo_final):
		tiempo = tiempo + dt


		ActualLeft = EncoderLeft.getTicks() - ActualLeft#Deberiamos leer los encoder aqui... no se como#
		ActualRight = EncoderRight.getTicks() - ActualRight#Deberiamos leer los encoder aqui... no se como#
		print(str(ActualRight))
		print(str(ActualLeft))

		ProportionalRight = ActualRight - InitialRight
		ProportionalLeft = ActualLeft - InitialLeft
		
		Proportion = ActualRight - ActualLeft

		DerivativeRight = ProportionalRight/dt
		DerivativeLeft = ProportionalLeft/dt






		OutputRight = SpeedRight + DerivativeRight * Kd - ProportionalRight * Kp #+ (DerivativeLeft * Kd + ProportionalLeft  * Kp)
		OutputLeft = SpeedLeft + DerivativeLeft * Kd + ProportionalLeft * Kp #-  (DerivativeRight * Kd + ProportionalRight  * Kp)

              

		if OutputRight >255:
			OutputRight = 255
		if OutputLeft >255:
			OutputLeft = 255
		if OutputRight <-255:
			OutputRight = -255
		if OutputLeft <-255:
			OutputLeft = -255
		print("Right "+str(OutputRight))
		print("Left "+str(OutputLeft))

		if OutputRight <= 0 and OutputLeft <= 0:
			pi.set_PWM_dutycycle(18, 0)
			pi.set_PWM_dutycycle(17, -OutputRight)

			pi.set_PWM_dutycycle(23, -OutputLeft)
			pi.set_PWM_dutycycle(22, 0)

		elif OutputRight <= 0 and OutputLeft > 0:
			pi.set_PWM_dutycycle(18, 0)
			pi.set_PWM_dutycycle(17, -OutputRight)

			pi.set_PWM_dutycycle(23, 0)
			pi.set_PWM_dutycycle(22, OutputLeft)

		elif OutputRight >0 and  OutputLeft <= 0:
			pi.set_PWM_dutycycle(18, OutputRight)
			pi.set_PWM_dutycycle(17, 0)

			pi.set_PWM_dutycycle(23, -OutputLeft)
			pi.set_PWM_dutycycle(22, 0)

		elif OutputRight >0 and  OutputLeft > 0:
			pi.set_PWM_dutycycle(18, OutputRight)
			pi.set_PWM_dutycycle(17, 0)

			pi.set_PWM_dutycycle(23, 0)
			pi.set_PWM_dutycycle(22, OutputLeft)




		InitialLeft = ActualLeft
		InitialRight = ActualRight
		sleep(dt)

	pi.set_PWM_dutycycle(18, 0)
	pi.set_PWM_dutycycle(17, 0)
	pi.set_PWM_dutycycle(23, 0)
	pi.set_PWM_dutycycle(22, 0)
	pi.stop()

PID(190, "Foward", 1)
