import time
import pigpio
from RPi import GPIO
import RotaryEncoder.py

#Codigo encoders#
"""clkr = 06
dtr = 12
clkl = 19
dtl = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(clkr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(clkl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dtl, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)"""
pi = pigpio.pi()
EncoderRight= Encoder(pi, 6, 12)
EncoderLeft= Encoder(pi, 19, 16)

#Codigo encoders#

pi.set_mode(18, pigpio.OUTPUT)
pi.set_mode(17, pigpio.OUTPUT)

pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(22, pigpio.OUTPUT)

Tiempo = raw_input("mete el tiempo")
Direction = raw_input("Foward, TurnRight, TurnLeft, Backwards")
velocidad = raw_input("belocidad")

def PID(Speed, Movement, t):
	dt = 0.3
	
	Kp = 0
	Kd = 0
	#Codigo encoders#
	"""

        counter = [0, 0]
        clkLastState = [GPIO.input(clkr), GPIO.input(clkl)]"""
        #/Codigo encoders#

        InitialLeft = EncoderLeft.getTicks#Deberiamos leer los encoder aqui... no se como#
	InitialRight = EncoderRight.getTicks #Deberiamos leer los encoder aqui... no se como#

	if Movement=="Foward":
		SpeedLeft = Speed
		SpeedRight = Speed
	elif Movement == "TurnRight":
		SpeedLeft  = Speed
		SpeedRight = -Speed
	elif Movement == "TurnLeft":
		SpeedLeft  = -Speed
		SpeedRight = Speed
	elif Movement == "Backwards":
		SpeedLeft = -Speed
		SpeedRight = -Speed

        tiempo=0
	while(True):
                tiempo = tiempo + dt
                #Codigo Encoders#
                """clkState = [GPIO.input(clkr), GPIO.input(clkl)]
                dtState = [GPIO.input(dtr), GPIO.input(dtr)]
                if clkState[0] != clkLastState[0]:
                        if dtState[0] != clkState[0]:
                                counter[0] -= 1
                        else:
                                counter[0] += 1
                        print(counter)
                if clkState[1] != clkLastState[1]:
                        if dtState[1] != clkState[1]:
                                counter[1] -= 1
                        else:
                                counter[1] += 1
                        print(counter)"""
                
                #Codigo Encoders#





		ActualLeft = counter[1]#Deberiamos leer los encoder aqui... no se como#
		ActualRight = counter[0]#Deberiamos leer los encoder aqui... no se como#

		ProportionalRight = ActualRight - InitialRight
		ProportionalLeft = ActualLeft - InitialLeft
		
		DerivativeRight = ProportionalRight/dt
		DerivativeLeft = ProportionalLeft/dt

		OutputRight = SpeedRight + DerivativeRight * Kd + ProportionalRight * Kp 
		OutputLeft = SpeedLeft + DerivativeLeft * Kd + ProportionalRight * Kp

		if OutputRight < 0:
                        pi.set_PWM_dutycycle(18, 0)
                        pi.set_PWM_dutycycle(17, OutputRight)

		#Aqui es donde le asignamos la salida a los motores... pero se me olvidaron esas funciones y los pinnes .. se las ponemos en el GIA#
		else:
                        pi.set_PWM_dutycycle(18, OutputRight)
                        pi.set_PWM_dutycycle(17, 0)
		#Aqui es donde le asignamos la salida a los motores... pero se me olvidaron esas funciones y los pinnes .. se las ponemos en el GIA#
		if OutputLeft < 0:
                        pi.set_PWM_dutycycle(23, OutputLeft)
                        pi.set_PWM_dutycycle(22, 0)
		#Aqui es donde le asignamos la salida a los motores... pero se me olvidaron esas funciones y los pinnes .. se las ponemos en el GIA#
		else :
                        pi.set_PWM_dutycycle(23, 0)
                        pi.set_PWM_dutycycle(22, OutputLeft)
		#Aqui es donde le asignamos la salida a los motores... pero se me olvidaron esas funciones y los pinnes .. se las ponemos en el GIA#

		InitialLeft = ActualLeft
		InitialRight = ActualRight
                #clkLastState = clkState#
		sleep(dt)
	pi.set_PWM_dutycycle(18, 0)
        pi.set_PWM_dutycycle(17, 0)
        pi.set_PWM_dutycycle(23, 0)
        pi.set_PWM_dutycycle(22, 0)
        pi.stop()

PID(Velocidad, Direction, Tiempo)
