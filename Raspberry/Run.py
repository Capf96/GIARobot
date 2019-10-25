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
grua = Grua(19, 16, 6, 12)
magnet = Magnet(23,22)

robot = Robot()
arduino = Arduino()

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
				return state.gardenBlocks[i][j]


def takeBlock(state, level):
	"""Procedimiento que toma un bloque.

	ARGUMENTOS:
	state: Estado del robot.
	"""
	
	# Nos acercamos lentamente al bloque
	robot.movStrUntBlock(15)

	"""
	# Subimos la grua
	grua.move(level)
	# Prendemos el electroiman
	magnet.on()
	# Nos movemos un poco hacia adelante
	robot.moveStraight(True, dist = 7, slow = True)
	grua.move(1)
	"""
	
	# Retrocedo un poco para bajar la grua
	motores.run(-17)
	sleep(0.8)
	motores.stop()
	# Actualizamos el estado
	#state.blockColor = robot.detectColor()
	state.blockColor = 0


def leaveBlock(state, green, level):
	"""Procedimiento para dejar un bloque lateral una vez que estas entre dos barcos.

	ARGUMENTOS:
	state: Estado del robot.
	green: Indica de que color es el bloque que lleva actualmente el robot
	"""
	
	# Subimos nivel y medio la grua para evitar que choque con los bloques que ya se encuentran en el barco
	grua.move(1.5)
	# Avanzamos hasta conseguir la linea negra
	robot.movStrUntObj(True, Slow = False, Line = True)
	# Nos movemos un poco hacia atras para poder alinearnos ----------------------
	robot.moveStraight(False, dist = 4, slow = False) 
	# Nos alineamos con la linea negra
	robot.align(20, True)
	# Retroceder un poco para volverte a aliniar
	robot.moveStraight(False, dist = 4, slow = False)
	# Retrocede y alineate con la linea negra 
	robot.align(25, True)
	# Movemos la grua
	newLvl = -level - 2.5 + (state.loadedBlocks[int(green)] % 3) 
	grua.move(newLvl)
	# Apagamos el iman
	magnet.off()
	# Retroceder hasta (mas o menos) el estado inicial ------------------
	robot.moveStraight(False, dist = 30, slow = False)
	# Bajo la grua
	newLvl = -(state.loadedBlocks[int(green)]%3)
	#pdb.set_trace()
	grua.move(newLvl)
	# Giramos alrededor de 90 grados dependiendo del color del bloque que habiamos cargado antes ---------------------------------------
	robot.turn(green, 230)
	# Actualizamos el estado
	state.loadedBlocks[int(green)] += 1
	state.blockColor = -1
	
	
def portToGarden(right, state):
	# Avanzar hasta la linea negra ------------------------------------
	robot.moveStrUntLine(25)
	# Avanzar un poco para facilitar el giro--------------------------
	robot.movStrUntTime(25, 0.4)
	# Girar hasta conseguir la linea negra  
	robot.turnUntLine(right) 
	# Seguir linea hasta conseguir la esquina
	robot.fllwLineUntObj(30, Corner = True)
	# Avanzar un poco para facilitar el giro -----------------------
	robot.movStrUntTime(25, 0.9)
	# Girar hasta conseguir la linea negra 
	robot.turnUntLine(not right) 
	# Seguir linea hasta conseguir el bloque
	robot.fllwLineUntObj(25, BlockL = right, BlockR = not right, dist = 30)
	# Seguir la linea un poco
	robot.fllwLineUntObj(25, Time = True, tm = 0.5)
	# Seguir linea hasta no conseguir el bloque
	robot.fllwLineUntObj(20, BlockL = right, BlockR = not right, No = True, dist = 30)
	# Gira alrdedor de 90 grados ------------------------------------
	robot.adjtAngle(17, not right, state, False)
	# Retrocede y alineate con la linea negra 
	#robot.align(25, False)	


def gardenToPort(right, state):
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(not right)
	# Seguir la linea hasta conseguir la esquina
	robot.fllwLineUntObj(30, Corner = True)
	# Avanzar un poco para facilitar el giro -----------------------
	robot.movStrUntTime(25, 0.9)
	# Girar hasta conseguir la linea negra
	robot.turnUntLine(right)
	# Seguir la linea poco tiempo para evitar problemas con la deteccion de la bifurcacion
	robot.fllwLineUntObj(30, Time = True, tm = 0.8)
	# Seguir la linea hasta conseguir una bifurcacion
	robot.fllwLineUntObj(30, Bifur = True)
	# Seguir la linea poco tiempo para evitar problemas con la deteccion de la bifurcacion
	robot.fllwLineUntObj(30, Time = True, tm = 0.8)
	# Seguir la linea hasta conseguir una bifurcacion
	robot.fllwLineUntObj(30, Bifur = True)
	# Gira alrdedor de 90 grados ------------------------------------
	robot.adjtAngle(17, not right, state, False)


def nearShip(right, state):
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
	gardenToPort(right, state)
	
	# Acercarse al barco correspondiente
	nearShip(right, state)
	# Dejamos el bloque y retroceder
	leaveBlock(state, green, level)
	
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

	magnet.on()
	while True:
		r = float(input("Indica R: "))		# 17.5
		robot.turnMagnet(20, clockwise = True, R = r)
		motores.stop()

	
	"""tpast = time() 
	epsilon = 0.1
	while True:
			dt = time() - tpast 
			if dt > epsilon:
				alphar = arduino.getAll()[11]
				print(alphar)
				alphao = alphar
				tpast = time() 
				"""
	# NO BORRAR NI COMENTAR
	robot.stop()
	##########
	
	


