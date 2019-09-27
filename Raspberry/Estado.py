"""Aqui se encuentra la clase Estado que guarda el estado de los bloques en la pista."""

class Estado(object):

	def __init__(self):
		self.gardenBlocks = [[4 for j in range(0, 4)] for k in range(0, 3)]
		"""Bloques restantes en los jardines.

			Primera coordenada: 0 -> Izquierda
								1 -> Derecha
								2 -> Central
			Segunda coordenada: [	00, 01, 
									10, 11	]
		Example:
			[
				[
					1, 0, 0, 3
				],

				[
					0, 2, 0, 0
				],

				[
					0, 0, 4, 0
				],
			]

		This is equal to
				|1|  |0|        |0|  |0|         |0|  |2|

				|0|  |3|        |4|  |0|         |0|  |0|
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


