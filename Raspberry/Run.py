"""Aqui se encuentran las funciones que permiten al robot realizar sus principales tareas."""

from time import sleep, time
from MotorControl import Motors
from Estado import Estado
from Robot import Robot
from electroiman import Magnet

robot = Robot()
motores = Motors(20,26)	# Pines GPIO20 y GPIO26
magnet = Magnet(23)
state = Estado()

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


def takeBlock(state):
	"""Procedimiento que toma un bloque.

	ARGUMENTOS:
	state: Estado del robot.
	"""

	# Definimos a que nivel subiremos la grua
	level = foundLevel(state)
	# Subir la grua
	grua.move(level)
	# Nos acercamos lentamente al bloque
	robot.movStrUntObj(True, Slow = True, BlockC = True, dist = 2)
	# Prendemos el electroiman
	magnet.on()
	# Retrocedo un poco para bajar la grua
	robot.moveStraight(False, dist = 6, slow = True)
	# Bajamos la grua
	grua.move(-level)
	# Actualizamos el estado
	state.blockColor = robot.detectColor()


def leaveBlock(state, green):
	"""Procedimiento para dejar un bloque lateral una vez que estas entre dos barcos.

	ARGUMENTOS:
	state: Estado del robot.
	green: Indica de que color es el bloque que lleva actualmente el robot
	"""

	# Definimos cuanto debe subir la grua
	grua.move(state.loadedBlocks[int(green)]%3)
	# Avanzamos hasta conseguir la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	# Nos movemos un poco hacia atras para poder alinearnos
	robot.moveStraight(False, dist = 4, slow = False)
	# Nos alineamos con la linea negra
	robot.align(20, True)
	# Apagamos el iman
	magnet.off()
	# Retroceder hasta (mas o menos) el estado inicial
	robot.moveStraight(True, dist = 30, slow = False)
	# Bajo la grua
	grua.move(-state.loadedBlocks[int(green)]%3)
	# Giramos alrededor de 90 grados dependiendo del color del bloque que habiamos cargado antes
	robot.turn(green, 130)
	# Actualizamos el estado
	state.loadedBlocks[int(green)] += 1
	state.blockColor = -1
	

def findLtrlBlock(right, state):
	"""Busca un bloque en alguno de los jardines laterales.
	
	ARGUMENTOS:
	right: True indica que buscara un bloque en el jardin derecho. False indica que lo buscara en el jardin izquierdo.
	state: Estado.
	"""
	
	# Avanzar hasta la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	"""
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 2, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(right) 
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(25, Corner = True)
	# Avanzar un poco para facilitar el giro
	robot.moveStraight(True, dist = 4, slow = False)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(not right) 
	# Seguir linea hasta conseguir el bloque
	robot.fllwLineUntObj(25, BlockL = right, BlockR = not right, dist = 30)
	# Avanzar un poco para una mejor alineacion con el bloque
	robot.fllwLineUntObj(25, Time = True, tm = 1.2)
	# Gira alrdedor de 90 grados
	robot.turn(not right, 110)
	# Retrocede y alineate con la linea negra 
	robot.align(20, False)
	# Tomamos el bloque correspondiente
	takeBlock(state)
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
	robot.fllwLineUntObj(25, Time = True, tm = 0.75)
	# Gira alrdedor de 90 grados
	robot.turn(not right, 110)
	# Retrocede y alineate con la linea negra 
	robot.align(20, False)
	# Avanzar hasta los barcos
	robot.moveStraight(True, dist = 30, slow = False)
	# Definimos si el bloque que llevamos es verde y azul
	green = (state.blockColor == 1)
	# Si el bloque es azul, avanzamos dependiendo de la cantidad de bloques azul llevados hasta ahora
	robot.moveStraight(True, dist = 30(1 + int(state.loadedBlocks[int(green)]/3)), slow = False)
	# Giramos alrededor de 90 grados
	robot.turn(green, 130)
	# Dejamos el bloque
	robot.leaveBlock(state, green)
	motores.stop()
	"""


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
	robot.fllwLineUntObj(25, Time = True, tm = 1.2)
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
	robot.fllwLineUntObj(25, Time = True, tm = 1)
	# Seguimos la linea hasta conseguir un bloque
	robot.fllwLineUntObj(25, BlockL = True, dist = 60)
	# Giramos alrededor de 90 grados
	robot.turn(green, 130)
	# Dejamos el bloque
	robot.leaveBlock(state, green)
	motores.stop()


if __name__ == "__main__":
	findLtrlBlock(True, state)
	

