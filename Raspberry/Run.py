"""Aqui se encuentran las funciones que permiten al robot realizar sus principales tareas."""

from time import sleep, time
from MotorControl import Motors
from Estado import Estado
from Robot import Robot
from Electroiman import Magnet
from ArduinoReader import Arduino
from StepperMotor import Grua

robot = Robot()
motores = Motors(26,20)	# Pines GPIO20 y GPIO26
magnet = Magnet(23)
state = Estado()
arduino = Arduino()
grua = Grua(19, 16, 6, 12)

def foundLevel(state):
	"""Funcion que indica cuanto debe subir la grua dependiendo de su estado.

	ARGUMENTOS:
	state: Estado del robot.
	"""

	for i in range(3):
		for j in range(4):
			if state.gardenBlocks[i][j] != 0:
				# Actualizamos el estado
				state.gardenBlocks[i][j] -= 1
				return state.gardenBlocks[i][j] - 1


def takeBlock(state, level):
	"""Procedimiento que toma un bloque.

	ARGUMENTOS:
	state: Estado del robot.
	"""
	
	# Nos acercamos lentamente al bloque
	robot.movStrUntObj(True, Slow = True, BlockC = True, dist = 3)
	# Subimos la grua
	grua.move(level)
	# Nos movemos un poco hacia adelante
	robot.moveStraight(True, dist = 5, slow = True)
	# Prendemos el electroiman
	magnet.on()
	grua.move(1)
	# Retrocedo un poco para bajar la grua
	robot.moveStraight(False, dist = 25, slow = True)
	# Bajamos la grua
	#grua.move(-level-1)
	# Actualizamos el estado
	#state.blockColor = robot.detectColor()
	state.blockColor = 0


def leaveBlock(state, green, level):
	"""Procedimiento para dejar un bloque lateral una vez que estas entre dos barcos.

	ARGUMENTOS:
	state: Estado del robot.
	green: Indica de que color es el bloque que lleva actualmente el robot
	"""

	# Avanzamos hasta conseguir la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	# Nos movemos un poco hacia atras para poder alinearnos
	robot.moveStraight(False, dist = 4, slow = False)
	# Nos alineamos con la linea negra
	robot.align(20, True)
	# Bajamos la grua
	grua.move(-level)
	# Apagamos el iman
	magnet.off()
	# Retroceder hasta (mas o menos) el estado inicial
	robot.moveStraight(True, dist = 30, slow = False)
	# Bajo la grua
	#grua.move(-state.loadedBlocks[int(green)]%3)
	# Giramos alrededor de 90 grados dependiendo del color del bloque que habiamos cargado antes
	robot.turn(green, 130)
	# Actualizamos el estado
	#state.loadedBlocks[int(green)] += 1
	#state.blockColor = -1
	

def findLtrlBlock(right, state):
	"""Busca un bloque en alguno de los jardines laterales.
	
	ARGUMENTOS:
	right: True indica que buscara un bloque en el jardin derecho. False indica que lo buscara en el jardin izquierdo.
	state: Estado.
	"""
	
	# Definimos a que nivel subiremos la grua
	level = foundLevel(state)
	# Avanzar hasta la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 6, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(right) 
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(30, Corner = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 4, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(not right) 
	# Seguir linea hasta conseguir el bloque
	robot.fllwLineUntObj(25, BlockL = right, BlockR = not right, dist = 30)
	# Avanzar un poco para una mejor alineacion con el bloque
	robot.fllwLineUntObj(25, Time = True, tm = 2.4)
	# Gira alrdedor de 90 grados
	robot.turn(not right, 110)
	# Retrocede y alineate con la linea negra 
	robot.align(25, False)
	# Tomamos el bloque correspondiente
	takeBlock(state, level)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(not right)
	# Seguir la linea hasta conseguir la esquina
	robot.fllwLineUntObj(25, Corner = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 4, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(right)
	# Seguir la linea poco tiempo para evitar problemas con la deteccion de la bifurcacion
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Seguir la linea hasta conseguir una bifurcacion
	robot.fllwLineUntObj(25, Bifur = True)
	# Seguir la linea hasta conseguir un bloque
	robot.fllwLineUntObj(25, BlockL = not right, BlockR = right, dist = 25)
	# Seguir la linea poco tiempo para quedar mas centrado entre los dos barcos
	robot.fllwLineUntObj(25, Time = True, tm = 1.9)
	# Gira alrdedor de 90 grados
	robot.turn(not right, 110)
	# Retrocede y alineate con la linea negra 
	robot.align(20, False)
	# Avanzar hasta los barcos
	robot.moveStraight(True, dist = 30, slow = False)
	# Definimos si el bloque que llevamos es verde y azul
	green = (state.blockColor == 1)
	# Si el bloque es azul, avanzamos dependiendo de la cantidad de bloques azul llevados hasta ahora
	robot.moveStraight(True, dist = 30*(1 + int((state.loadedBlocks[int(green)])/3)), slow = False)
	# Giramos alrededor de 90 grados
	robot.turn(green, 210)
	# Dejamos el bloque
	robot.leaveBlock(state, green, level)
	motores.stop()


def findCntrlBlock(state):
	"""Busca un bloque en alguno de los jardines laterales.
	
	ARGUMENTOS:
	state: Estado del robot.
	"""

	# Avanzar hasta la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 2, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(True)
	# Seguir la linea poco tiempo para evitar problemas con la deteccion de la bifurcacion
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Seguir la linea hasta conseguir una bifurcacion
	robot.fllwLineUntObj(25, Bifur = True)
	# Seguir la linea poco tiempo para facilitar el giro
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Giramos alrededor de 45 grados
	robot.turn(False, 65)
	# Giramos hasta conseguir linea
	robot.turnUntLine(False) 
	# Seguimos la linea hasta conseguir un bloque
	robot.fllwLineUntObj(25, BlockL = True, dist = 25)
	# Avanzar un poco para una mejor alineacion con el bloque
	robot.fllwLineUntObj(25, Time = True, tm = 1)
	# Gira alrdedor de 90 grados
	robot.turn(False, 110)
	# Retrocede y alineate con la linea negra 
	robot.align(20, False)
	# Tomamos el bloque correspondiente
	takeBlock(state)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(False)
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(25, Corner = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 4, slow = False)
	# Giramos hasta conseguir linea
	robot.turnUntLine(False) 
	# Seguir la linea poco tiempo para evitar problemas con la deteccion del bloque
	robot.fllwLineUntObj(25, Time = True, tm = 3)
	# Seguimos la linea hasta conseguir un bloque
	robot.fllwLineUntObj(25, BlockL = True, dist = 60)
	# Giramos alrededor de 90 grados
	robot.turn(green, 130)
	# Dejamos el bloque
	robot.leaveBlock(state, green)
	motores.stop()


if __name__ == "__main__":
	arduino.getAll()
	magnet.off()
	grua.move(-3)
	#findLtrlBlock(True, state)

