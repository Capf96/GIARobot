from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Robot import Robot
from Electroiman import Magnet
from StepperMotor import Grua
robot = Robot()
motores = Motors(20,26) # Pines GPIO27 y GPIO18
arduino = Arduino()
magnet = Magnet(23)
grua = Grua(19, 16, 06, 12)
pwm = 15
motores.stop()

magnet.off()

grua.stepper(0, 0.001)
