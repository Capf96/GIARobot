from MotorControl import Motor
from RobotControl import Robot
import pigpio

import time
from electroIman import Magnet
from Sensores import QTR, Sensor
from grua import Grua
from UltraSonic import UltrasonicS


#Para una vuelta de 90 grados se necesitan que las ruedas giren alrededor de 5700 ticks

motorL = Motor("Motor Izquierdo", 18, 17, 26, 20)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)
iman = Magnet(27)
qtr = QTR([0,1,2,3,4,5,6,7])
usDer = UltrasonicS(21, 4)
usIzq = UltrasonicS(21, 24)
usIm  = UltrasonicS(21, 25)
grua = Grua(10, 9, 7, 8, usIm)


robot = Robot(motorL, motorR, iman, qtr, usIzq, usDer)



"""
robot.magnet.on()
sleep(1)
raw_input("el que le de paly me mama el bichoo")
grua.subir()
sleep(7)
grua.parar()
raw_input("volviste y dle diste play y me volviste a mamar el bivho")
robot.getToLineF(30)
sleep(1)
grua.bajar()
sleep(7)
grua.parar()
robot.magnet.off()
robot.backward(20,50)
"""


#Parametros del PID#
kp = 0.75 # Constante Proporcional
ki = 0	  # Constante Integral
kd = 0	  # Constante Diferencial

#Parametros dl pwm
pwm = 35

#Vairables de tiempo
dt = 0        # Diferencial de tiempo. *Es el tiempo que espera el sistema para aplciar de nuevo los calculos de ajuste del PID.*
epsilon = 0.7
timepast = 0 
integral=0
d = 20
k = 0
while k < 5:
	# Mide el tiempo actual
	timenow  = time.time()	
	# Calcula diferencia entre el teimpo actual y el pasado
	dt = timenow - timepast
	#Si se supera el epsilon se hace el calculo del PID
	if dt >= epsilon:
		robot.seguirLinea(pwm, kp, ki, kd, integral)
		timenow = timepast
	k += 1
	time.sleep(1)

robot.stop()
