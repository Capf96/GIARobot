from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Estado import Estado
from Robot import Robot
from Electroiman import Magnet
from StepperMotor import Grua
from Run import findLtrlBlock, findCntrlBlock

robot = Robot()
motores = Motors(20,26) # Pines GPIO27 y GPIO18
arduino = Arduino()
state = Estado()
grua = Grua(19, 16, 18, 17)
magnet = Magnet(23)


magnet.on()
grua.stepper(100, 0.003)
sleep(3)
magnet.off()


# Para cuando el robot este listo.
"""
if __name__ == "__main__":
	blocks = 0
	while blocks < 36:
		if blocks < 24:
			findLtrlBlock(blocks < 12, state)
		else:
			findCntrlBlock()

	print("CHANGCITA! LO LOGRE!!!!!.")
"""
