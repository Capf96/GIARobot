"""Aqui se encuentra la clase Arduino que nos permite obtener el valor de todos los sensores."""

import serial

ser = serial.Serial('/dev/ttyACM0', 57600)
ser.reset_input_buffer()

class Arduino(object):

	def getAll(self):
		"""Obtenemos un arreglo con el valor de todos los sensores.
			Los primeros 8 valores representan el QTR.
			El noveno es la posicion.
			Los siguientes 3 representan el ultrasonido izquierdo, central y derecho respectivamente.
			Los siguientes 6 son los valores del gyroscopio
		"""
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getAll()
			for i in range(len(s)):
				s[i]=float(s[i])
				
			pos = 0; suma = 0
			for i in range(8):
				suma += s[i]
				pos += i*1000*s[i]
			
			s[8] = pos/suma
			s[11] = 360 - s[11]
		return s


	def getQTR(self):
		"""Obtenemos un arreglo con los 8 valores del QTR."""
		
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getQTR()
			for i in range(8):
				s[i]=int(s[i])
		return s[0:8]
		
		
	def getUltraL(self):
		"""Obtenemos el valor del ultrasonido izquierde."""
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getUltraL()
			return float(s[9])
		
		
	def getUltraR(self):
		"""Obtenemos el valor del ultrasonido derecho."""
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getUltraR()
			return float(s[11])
	
		
	def getAverageQTR(self):
		
		"""Obtenemos la posicion (promedio) entre los 8 sensores QTR."""
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getAverageQTR()
			
			pos = 0; suma = 0
			for i in range(8):
				suma += int(s[i])
				pos += i*1000*int(s[i])
			
			return pos/suma

		
	def getFullGyro(self):
		"""Obtenemos un arreglo con los 5 valores del giroscopio."""
		
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.getFullGyro()
				
			for i in range(12, 17):
				s[i]=float(s[i])
		return s[12:17]
	
	
	def gyro(self):
		""" Obtenemos el valor del giroscopio """
		data = list(ser.readline())
		if data:
			result = str(ser.readline().strip())
			result = result.replace('b', '')
			result = result.replace("'", '')
			s = result.split(' ')
			if(len(s)!=14):
				return self.gyro()
			for i in range(len(s)):
				s[i]=float(s[i])
				
			return float(s[len(s) - 1])
	



if __name__ == "__main__":
	arduino = Arduino()
	ser.reset_input_buffer()
	while True:
		print(arduino.getAll())

