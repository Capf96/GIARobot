from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Robot import Robot

robot = Robot()
motores = Motors(17,27) # Pines GPIO17 y GPIO27
arduino = Arduino()


pwm = 30
motores.stop()

arduino.getQTR()
robot.movStrUntObj(False, Line = True)

motores.stop()
