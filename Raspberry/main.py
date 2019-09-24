from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Robot import Robot
#from electroiman import Magnet

robot = Robot()
motores = Motors(20,26) # Pines GPIO27 y GPIO18
arduino = Arduino()

arduino.getAll()

pwm = 15
motores.stop()

"""while True:
	print arduino.getQTR()"""	
	
robot.movStrUntObj(True, Slow = False, Line = True)
robot.moveStraight(True, dist = 2, slow = False)
robot.turnUntLine(True)
robot.fllwLineUntObj(25, Corner = True)
robot.moveStraight(True, dist = 4, slow = False)
robot.turnUntLine(False)
robot.fllwLineUntObj(25, BlockL = True, dist = 25)
robot.fllwLineUntObj(25, Time = True, tm = 1.2)
robot.turn(False, 110)
robot.align(10, False)
robot.moveStraight(True, dist = 6, slow = False)
sleep(2)
robot.turnUntLine(False)
robot.fllwLineUntObj(25, Corner = True)
robot.moveStraight(True, dist = 4, slow = False)
robot.turnUntLine(True)
robot.fllwLineUntObj(25, Time = True, tm = 0.5)
robot.fllwLineUntObj(25, Bifur = True)
robot.fllwLineUntObj(25, BlockR = True, dist = 25)
robot.fllwLineUntObj(25, Time = True, tm = 0.75)
robot.turn(False, 110)
robot.align(10, False)
robot.moveStraight(True, dist = 60, slow = False)

motores.stop()

