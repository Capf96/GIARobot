
# Importaciones de modulos oficiales
from time import sleep, time

# Importaciones de nuestros archivos .py
from MotorControl import Motors
from Estado import Estado
from Robot import Robot
#from electroiman import Magnet

robot = Robot()
motores = Motors(20,26)	# Pines GPIO27 y GPIO18
state = Estado()

def dejarBloque():
	robot.moveStraight(True, dist = 60, slow = False)
	
	tr = estado.blockColor # Solo si son Bloques azules o verdes
	robot.turn(tr, 135)
	robot.movStrUntObj(True, Slow = False, Line = True)
	robot.moveStraight(False, dist = 4, slow = False)
	robot.align(10, True)
	

def findLtrlBlock(right, state):
	"""Busca un bloque en alguno de los jardines laterales.
	
	ARGUMENTOS:
	right: True indica que buscara un bloque en el jardin derecho. False indica que lo buscara en el jardin izquierdo.
	state: Estado 
	"""
	
	# Avanzar hasta la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
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
	robot.align(15, False)
	
	""" Aqui iria acercarse y tomar el bloque, asi como la deteccion de color. Por ahora 
		solo avanzara un poco:
		
	# Nos acercamos lentamente al bloque
	robot.toNearBlock()
	# Tomamos el bloque
	robot.takeBlock(state)
	# Guardamos el color del bloque
	state.blockColor = robot.detectColor()
	# Bajamos la grua
	robot.level(0)
	# Nos movemos un poco hacia atras para facilitar el giro
	robot.moveStraight(False, dist = 2, slow = True)
	"""
	robot.moveStraight(True, dist = 6, slow = False)	# Eliminar
	sleep(2)	# Eliminar
	
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
	robot.align(15, False)
	# Avanzar hasta los barcos
	robot.moveStraight(True, dist = 30, slow = False)
	
	""" Aqui iria avanzar dependiendo de la cantidad de bloques llevados del mismo color, 
		dejar el bloque y regresar al estado inicial:
		
	# Esta variable nos indicara que tanto avanzar y hacia donde girar
	green = state.blockColor == 1
	# Si el bloque es azul, avanzamos dependiendo de la cantidad de bloques azul llevados hasta ahora
	robot.moveStraight(True, dist = 30(1 + int(state.loadedBlocks[int(green)]/3)), slow = False)
	# Giramos alrededor de 90 grados
	robot.turn(green, 130)
	# Nos alineamos con la linea del barco
	robot.align(15, True)
	# Dejamos el bloque
	robot.goDownBlock()
	# Nos movemos un poco hacia atras para facilitar el giro
	robot.moveStraight(False, dist = 2, slow = True)
	# Giramos alrededor de 90 grados
	robot.turn(green, 130)
	"""

	motores.stop()


def buscarBloqueCentro(dirA = True, dirB = True):
	# Acercarse a la linea y avanzar un poco
	robot.movStrUntObj(True, Slow = False, Line = True)
	robot.moveStraight(True, dist = 2, slow = False)

	robot.turnUntLine(dirA) 
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	robot.fllwLineUntObj(25, Bifur = True)
	robot.moveStraight(True, dist = 4, slow = False)
	robot.turn(not dirA, 65)
	robot.turnUntLine(not dirA) 

	robot.fllwLineUntObj(25, Block = dirB, dist = 25)

	robot.fllwLineUntObj(25, Time = True, tm = 1.2)
	robot.turn(not dirA, 110) 
	robot.align(10, False)
	robot.moveStraight(True, dist = 6, slow = False)
	sleep(2)
	robot.turnUntLine(not dirA)
	robot.fllwLineUntObj(25, Corner = True)
	robot.moveStraight(True, dist = 4, slow = False)
	robot.turnUntLine(dirA)
	robot.fllwLineUntObj(25, Time = True, tm = 1)

	robot.turn(not dirA, 110)
	robot.align(10, False)

	dejarBloque()
	motores.stop()

findLtrlBlock(True)

"""
# Decision dependiendo de la cantidad de bloques en cada jardin, dirA : direccionde giros, dirB direccion del bloque
if estado.gardenBlocks[0] < maxBlocks:
	buscarBloqueLateral(True,True)
elif estado.gardenBlocks[2] < maxBlocks:
	buscarBloqueLateral(False,False)
else:
	buscarBloqueCentro()
"""
	

