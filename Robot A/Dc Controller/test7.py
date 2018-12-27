from MotorControl import Motor
from RobotControl import Robot
import pigpio
from time import sleep

#Para una vuelta de 90 grados se necesitan que las ruedas giren alrededor de 5700 ticks

motorL = Motor("Motor Izquierdo", 18, 17, 6, 12)
motorR = Motor("Motor Derecho", 23, 22, 16, 19)

robot = Robot(motorL, motorR)

robot.turnRight(180, 50)






