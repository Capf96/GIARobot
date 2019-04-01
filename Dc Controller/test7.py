from MotorControl import Motor
from RobotControl import Robot
import pigpio
from time import sleep
from electroIman import Magnet
from Sensores import QTR, Sensor


#Para una vuelta de 90 grados se necesitan que las ruedas giren alrededor de 5700 ticks

motorL = Motor("Motor Izquierdo", 18, 17, 26, 20)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)
iman = Magnet(27)
qtr = QTR([0,1,2,3,4,5,6,7])

robot = Robot(motorL, motorR, iman, qtr)

#robot.fowardB(50, 50)
sleep(0.1)
#robot.backwardB(30, 50)
#robot.getToLineB(50)
robot.getToLineF(40)
#robot.fowardL(50)
#robot.getToLineF(50)
#robot.fowardB(8, 50)










