"""Aqui se encuentran las funciones que permiten al robot realizar sus principales tareas."""

from ArduinoReader import Arduino
from MotorControl import Motors
from StepperMotor import Grua
from Electroiman import Magnet
from Encoder import Encoder
from Estado import Estado
from Switch import Switch
from Robot import Robot

from multiprocessing import Process
from time import sleep, time
from pdb import set_trace

import pigpio
import serial

#Componentes del Robot
suicheD = Switch(9,10) #Suiche izquierdo	
suicheI = Switch(24,25)	#Suiche Derecho
motores = Motors(26,20)	# Pines GPIO20 y GPIO26
grua = Grua(19, 16, 18, 17)
magnet = Magnet(23,22)

robot = Robot()
arduino = Arduino()

def foundLevel(state):
	"""Funcion que indica cuanto debe subir la grua dependiendo de su estado.

	ARGUMENTOS:
	state: Estado del robot.
	"""

	for i in [0,2,1]:
		for j in range(4):
			if state.gardenBlocks[i][j] != 0:
				# Actualizamos el estado
				state.gardenBlocks[i][j] -= 1
				return state.gardenBlocks[i][j]


def takeBlock(state, level):
	"""Procedimiento que toma un bloque.

	ARGUMENTOS:
	state: Estado del robot.
	level: nivel al que tiene que agarrar el bloque 
	"""
	
	# Nos acercamos lentamente al bloque
	robot.movStrUntBlock(17)
	input("Subiendo la grua " + str(level) + " niveles")
	# Subir grua
	grua.move(level)
	# Avanzar un poco
	robot.movStrUntTime(20, 0.3)
	# Prender iman
	magnet.on()
	sleep(1)
	# Retrocedo un poco
	motores.run(-17)
	sleep(0.5)
	motores.stop()
	grua.move(3-level)
	# Actualizamos el estado
	#state.blockColor = robot.detectColor()
	state.blockColor = 0
	
	
def portToGarden(right, state):
	"""Funcion para ir desde los barcos a los jardines.
	
	PARAMETROS:
	right: Indica si se debe ir al bloque derecho o no.
	state: Estado del robot.
	"""
	
	# Seguir linea hasta conseguir bifurcacion
	robot.fllwLineUntObj(30, Bifur = True)
	# Girar hasta conseguir la linea negra 
	robot.turnEncoders(20, right, static = False, ang = 80)
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(30, Corner = True)
	# Avanzar un poco para facilitar el giro
	robot.movStrUntTime(25, 0.6)
	# Girar hasta conseguir la linea negra 
	robot.turnUntLine(not right) 
	# Seguir la linea un poco
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Seguir linea hasta conseguir el bloque
	robot.fllwLineUntObj(25, BlockL = right, BlockR = not right, dist = 30)
	# Seguir la linea un poco
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Seguir linea hasta no conseguir el bloque
	robot.fllwLineUntObj(15, BlockL = right, BlockR = not right, No = True, dist = 30)
	# Gira alrdedor de 90 grados
	robot.turnEncoders(20, not right, ang = 80)


def gardenToPort(right, state, level):
	"""Funcion para ir de los jardines a los barcos
	
	ARGUMENTOS:
	right: Indica si te encuentras en el jardin derecho.
	state: Estado del robot.
	"""
	
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(not right)
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(30, Corner = True)
	# Bajamos la grua
	grua.move(-3)
	# Avanzar un poco para facilitar el giro
	robot.movStrUntTime(25, 0.6)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(right)
	# Seguir la linea un poco
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	
	if state.loadedBlocks[state.blockColor] < 3:
		# Numero de bloques dejados 
		numBlocks = state.loadedBlocks[0] + state.loadedBlocks[1]
		# Dejar el bloque al otro lado del jardin en caso de que este al otro lado
		if (numBlocks<12 and state.blockColor==1) or (numBlocks>=12 and state.blockColor==0):
			# Seguir linea hasta conseguir bifurcacion
			robot.fllwLineUntObj(25, Bifur = True)
			# Seguir la linea un poco
			robot.fllwLineUntObj(25, Time = True, tm = 1)
			# Seguir linea hasta conseguir bifurcacion
			robot.fllwLineUntObj(25, Bifur = True)
			# Seguir la linea un poco
			robot.fllwLineUntObj(25, Time = True, tm = 1)
			# Seguir linea hasta conseguir bifurcacion
			robot.fllwLineUntObj(25, Bifur = True)
		# Seguir linea hasta conseguir el bloque
		robot.fllwLineUntObj(25, BlockL = not right, BlockR = right, dist = 30)
		# Seguir la linea un poco
		robot.fllwLineUntObj(25, Time = True, tm = 1)
		# Gira alrdedor de 90 grados
		robot.turnEncoders(20, not right, ang = 100, static = False)
		motores.stop()
		# Subimos la grua
		input("Subiendo la grua " + str(state.loadedBlocks[state.blockColor]%3+1) + " niveles")
		grua.move(state.loadedBlocks[state.blockColor]%3 + 1)
		# Alinearse
		robot.align(18, True)
		# Bajamos la grua
		grua.move(-1)
		# Apagamos el iman
		magnet.off()
		# Retrocedo un poco
		motores.run(-17)
		sleep(1.25)
		motores.stop()
		# Bajamos la grua
		grua.move(-state.loadedBlocks[state.blockColor]%3)
		# Gira alrdedor de 180 grados
		robot.turnEncoders(20, not right, ang = 190, static = True)
	else:
		# Seguir linea hasta conseguir bifurcacion
		robot.fllwLineUntObj(25, Bifur = True)
		# Seguir la linea un poco
		robot.fllwLineUntObj(25, Time = True, tm = 1)
		# Seguir linea hasta conseguir bifurcacion
		robot.fllwLineUntObj(25, Bifur = True)
		# Girar
		robot.turnEncoders(25, not right, ang = 95, static = False)
		
		n = int(state.loadedBlocks[state.blockColor]/3)
		
		while n:
			# Seguir linea hasta conseguir el bloque
			robot.fllwLineUntObj(20, BlockL = right, BlockR = not right, dist = 40)
			# Seguir linea hasta no conseguir el bloque
			robot.fllwLineUntObj(15, BlockL = right, BlockR = not right, No = True, dist = 40)
			n -= 1
		
		# Seguir la linea un poco
		robot.fllwLineUntObj(25, Time = True, tm = 1.4)
		ship = (state.blockColor == 1)
		# Gira alrdedor de 90 grados
		robot.turnEncoders(20, ship, ang = 90, static = False)
		motores.stop()
		# Subimos la grua
		input("Subiendo la grua " + str(state.loadedBlocks[state.blockColor]%3+1) + " niveles")
		grua.move(state.loadedBlocks[state.blockColor]%3 + 1)
		robot.align(18, True)
		# Bajamos la grua
		grua.move(-1)
		# Apagamos el iman
		magnet.off()
		# Retrocedo un poco
		motores.run(-17)
		sleep(0.5)
		motores.stop()
		# Bajamos la grua
		grua.move(-state.loadedBlocks[state.blockColor]%3)
		# Gira alrdedor de 180 grados
		robot.turnEncoders(20, not right, ang = 190)
		# Alinearse
		robot.align(18, True)
		# Gira alrdedor de 90 grados
		robot.turnEncoders(20, right, ang = 100, static = False)
		
	state.loadedBlocks[state.blockColor] += 1
	# Actualizamos el estado
	state.colorBlock = -1
	
	print(state.gardenBlocks)
	print(state.loadedBlocks)
				

def nearShip(right, state):
	"""
	# Seguir linea hasta conseguir el bloque
	robot.fllwLineUntObj(25, BlockL = not right, BlockR = right, dist = 30)
	# Girar
	robot.turnEncoders(25, not right, ang = 100, static = False)
	"""
	# Avanzar hasta los barcos -----------------------------------
	robot.moveStraight(True, dist = 30, slow = False)
	# Definimos si el bloque que llevamos es verde y azul
	green = (state.blockColor == 1)
	# Si el bloque es azul, avanzamos dependiendo de la cantidad de bloques azul llevados hasta ahora -------------------------------------
	robot.moveStraight(True, dist = 30*(1 + int((state.loadedBlocks[int(green)])/3)), slow = False)
	# Giramos alrededor de 90 grados -----------------------------------------------------------
	robot.turn(green, 230)


def findLtrlBlock(right, state):
	"""Busca un bloque en alguno de los jardines laterales.
	
	ARGUMENTOS:
	right: True indica que buscara un bloque en el jardin derecho. False indica que lo buscara en el jardin izquierdo.
	state: Estado.
	"""
	# Definimos a que nivel subiremos la grua
	level = foundLevel(state)
	
	# Movernos del Puerto al jardin
	portToGarden(right, state)
	
	# Tomamos el bloque correspondiente y retrocedemos
	takeBlock(state, level)
	
	# Movernos del jardin al puerto
	gardenToPort(right, state, level)

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
	# NO BORRAR NI COMENTAR 
	magnet.off(); arduino.getAll(); state = Estado()
	print ("############################### VERIFICA QUE LA GRUA NO VA A BAJAR MAS DE LA CUENTA #########################################")
	print ("Si ya verificaste que la grua no bajara mas alla del nivel 0, escribe 'c' y presiona enter. En caso contrario presiona control+c.")
	set_trace()
	grua.move(0)
	########################

	m = 32
	while m:
		right = (m > 16)
		findLtrlBlock(right, state)

	
	# NO BORRAR NI COMENTAR
	robot.stop()
	##########
	
	


