from Sensores import QTR, Sensor
import serial
from time import sleep
from MotorControl import Motor
from RobotControl import Robot
import pigpio
from electroIman import Magnet

motorL = Motor("Motor Izquierdo", 18, 17, 6, 12)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)
iman = Magnet(27)
qtr = QTR([0,1,2,3,4,5,6,7])

#robot = Robot(motorL, motorR, iman, qtr)
while True :
	print(qtr.getValues())

