class Estado(object):

	def __init__(self):
		self.gardenBlocks = [[[4 for i in range(0, 2)] for j in range(0, 2)] for k in range(0, 3)]
		"""Bloques restantes en los jardines.

			Primera coordenada: Jardin
			Segunda coordenada: Fila
			Tercera coordenada: Columna
		Example:
			[
				[
					[ 1, 0],
					[ 0, 3]
				],

				[
					[ 0, 2],
					[ 0, 0]
				],

				[
					[ 0, 0],
					[ 4, 0]
				],
			]

		This is equal to
				|1|  |0|        |0|  |2|         |0|  |0|

				|0|  |3|        |0|  |0|         |4|  |0|
		"""

		self.loadedBlocks = [0, 0, 0]
		"""Bloques ya cargados.

			Primera coordenada: Bloques en el barco azul.
			Segunda coordenada: Bloques en el barco verde.
			Tercera coordenada: Bloques en el tren (rojo).
		"""

		self.blockColor = -1
		"""Indica el color del bloque que lleva actualmente.
			
			-1 -> No lleva ningun bloque
			0 -> Azul
			1 -> Verde
			2 -> Rojo
		"""

		self.state = [False, False, 0]
		"""Indica hacia donde se debe ir el robot.

			Primera coordenada: Ir hacia los barcos (True) o al tren (False)
			Segunda coordenada: Izquierda (True) o derecha (False)
			Tercera coordenada: Indica que tanto debe avanzar al ir a los barcos
		"""


	def decision(self):
		"""Indica al robot hacia donde debe moverse inmediatamente despues de tomar un bloque"""

		if self.blockColor == 0:
			nivel = int(self.loadedBlocks[0] / 4)
			self.state = [True, True, nivel + 1]

		elif self.blockColor == 1:
			nivel = int(self.loadedBlocks[1] / 4)
			self.state = [True, False, nivel + 1]

		elif self.blockColor == 2:
			direc = bool(self.loadedBlocks[2])
			self.state = [False, not direc, 0]


