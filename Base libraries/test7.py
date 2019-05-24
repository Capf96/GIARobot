from MotorControl import Motor
from RobotControl import Robot
import pigpio
from time import sleep
from electroIman import Magnet
from Sensores import QTR, Sensor
from grua import Grua


#Para una vuelta de 90 grados se necesitan que las ruedas giren alrededor de 5700 ticks

motorL = Motor("Motor Izquierdo", 18, 17, 26, 20)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)
iman = Magnet(27)
qtr = QTR([0,1,2,3,4,5,6,7])
grua = Grua(25, 24)

robot = Robot(motorL, motorR, iman, qtr)



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

#robot.aling(30)
#robot.forward(5,50)
#robot.getToLineF(30)
#obot.aling(30)
#robot.backward(10,50)
#robotot.turnLeft(180, 50)
#robot.getToLineF(30)
#robot.aling(30)
robot.magnet.on()
sleep(3)
robot.magnet.off()





