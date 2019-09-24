from time import sleep, time
from MotorControl import Motors
from ArduinoReader import Arduino
from Estado import Estado
from Robot import Robot
from electroiman import Magnet
from StepperMotor import Grua

robot = Robot()
motores = Motors(20,26) # Pines GPIO27 y GPIO18
arduino = Arduino()
estado = Estado()
grua = Grua(19, 16, 06, 12)
magnet = Magnet(23)

magnet.on()
grua.stepper(3000, 0.003)
sleep(3)
magnet.off()
	

"""
# Decision dependiendo de la cantidad de bloques en cada jardin, dirA : direccionde giros, dirB direccion del bloque
if estado.gardenBlocks[0] < maxBlocks:
	buscarBloqueLateral(True,True)
elif estado.gardenBlocks[2] < maxBlocks:
	buscarBloqueLateral(False,False)
else:
	buscarBloqueCentro()
"""
	

