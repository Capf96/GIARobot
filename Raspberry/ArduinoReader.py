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
			Los ultimos dos son izquierdo y derecho respectivamente.
		"""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(len(s)):
				s[i]=float(s[i])
				
			pos = 0; suma = 0
			for i in range(8):
				suma += s[i]
				pos += i*1000*s[i]
			
			s[8] = pos/suma
		return s


	def getQTR(self):
		"""Obtenemos un arreglo con los 8 valores del QTR."""
		
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(8):
				s[i]=int(s[i])
		return s[0:8]
		
		
	def getUltraL(self):
		"""Obtenemos el valor del ultrasonido izquierde."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[9])
		
		
	def getUltraR(self):
		"""Obtenemos el valor del ultrasonido derecho."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[11])
	
	
	def getUltraC(self):
		"""Obtenemos el valor del ultrasonido central."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[10])
		
		
	def getEncoderL(self):
		"""Obtenemos el valor del encoder izquierdo."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return 360 - float(s[12])
		
		
	def getEncoderR(self):
		"""Obtenemos el valor del encoder derecho."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[13])
		
		
	def getAverageQTR(self):
		"""Obtenemos la posicion (promedio) entre los 8 sensores QTR."""
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			
			pos = 0; suma = 0
			for i in range(8):
				suma += int(s[i])
				pos += i*1000*int(s[i])
			
			return pos/suma



if __name__ == "__main__":
	arduino = Arduino()
	ser.reset_input_buffer()
	while True:
		print(arduino.getAverageQTR())

