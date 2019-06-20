from MotorControl import Motor
from RobotControl import Robot
import pigpio

import time
from electroIman import Magnet
from Sensores import QTR, Sensor
from grua import Grua
from UltraSonic import UltrasonicS
from multiprocessing import Process

#Para una vuelta de 90 grados se necesitan que las ruedas giren alrededor de 5700 ticks

motorL = Motor("Motor Izquierdo", 18, 17, 26, 20)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)
iman = Magnet(27)
qtr = QTR([0,1,2,3,4,5,6,7])
usDer = UltrasonicS(21, 4)
usIzq = UltrasonicS(21, 24)
usIm  = UltrasonicS(21, 25)
usLIzq = UltrasonicS(21, 11)
grua = Grua(10, 9, 7, 8, usIm)

pwm = 35


robot = Robot(motorL, motorR, iman, qtr, usIzq, usDer, usLIzq)

# Llegar hasta la linea, y luego dirigirse a la esquina
robot.getToLineF(pwm)
robot.forward(9,pwm)
robot.turnUntilLine(pwm, True)
robot.followLineCorner(pwm, False, False)

# Girar hpara quedar paralelo al jardin lateral y avanzar hasta conseguir bloque
robot.turnUntilLine(pwm, False)
robot.followLineUntilBlock(pwm)

# Girar hacia el bloque y alinearse con el
robot.turnLeft(90, pwm)
robot.backToLine(pwm)
robot.backward(14, pwm)
robot.aling(pwm)
robot.alingBlock(pwm)

# Acercarse al bloque, tomarlo y retroceder
grua.nivel(2)
robot.getNearBlock(pwm)
iman.on()
grua.subirMax()
robot.backward(14, pwm)
grua.bajar()
time.sleep(6)
grua.parar()

# Girar de regreso hacia la linea, y luego seguir la linea hasta la esquina
robot.turnUntilLine(pwm, False)
robot.followLineCorner(pwm, False, False)

# Girar la esquina y seguir la linea hasta conseguir bloque
robot.turnUntilLine(pwm, True)
robot.followLineUntilBlock(pwm)

# Girar hacia el bloque y alinearse con el
robot.turnLeft(90, pwm)
robot.getToLineF(pwm)
robot.backward(3, pwm)
robot.aling(pwm)

# Acercarse al bloque y apilar el que tiene
grua.subirMax()
robot.getNearBlock(pwm)
grua.bajar()
time.sleep(6)
iman.off()
grua.nivel(0)

# Retroceder un poco y girarse hacia los jardines para repetir el proceso
robot.backward(3, pwm)
robot.turnLeft(180, pwm)

"""
print("debo avanzar")
robot.turnUntilLine(pwm, True)
print("memecito")
robot.followLineCorner(pwm, False, False)
robot.turnUntilLine(pwm, False)
robot.followLineUntilBlock(pwm)
"""
"""
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
#robot.aling(35)


