from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Robot import Robot
from Electroiman import Magnet

robot = Robot()
motores = Motors(20,26) # Pines GPIO27 y GPIO18
arduino = Arduino()
magnet = Magnet(23)

arduino.getQTR()

pwm = 15
motores.stop()
magnet.off()
