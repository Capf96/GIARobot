from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Robot import Robot

robot = Robot()
motores = Motors(17,27) # Pines GPIO17 y GPIO27
arduino = Arduino()


pwm = 30
motores.stop()

sleep(2)

while True:
	a  = input("")
	motores.run(a)

#robot.followLine(30)
"""	

robot.movStrUntObj(True, Slow = False, Line = True)
robot.moveStraight(True, dist = 7, slow = False)
robot.turnUntLine(True)
robot.fllwLineUntObj(pwm, Corner = True)
robot.moveStraight(True, dist = 7, slow = False)
robot.turnUntLine(False)
robot.fllwLineUntObj(pwm, BlockL = True, dist = 25)

motores.stop()
"""
