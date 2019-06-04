import serial
ser = serial.Serial('/dev/ttyUSB0', 57600)
ser.flushInput()
import time
class Sensor(object):
	def __init__(self, pos):
		self.pos = pos #posicion de la lectura del sensor en la lista que manda el arduino
		ser.write('A'+"\n")

	def getValue (self):
		ser.write('A'+"\n")
		ser.write('A'+"\n")
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(',')
		return int(s[self.pos])
		
		
class QTR(object):
	def __init__(self, positions):
		print("Calibrando")
		ser.write('A'+"\n")
		ser.write('A'+"\n")
		time.sleep(3)
		w = raw_input("Calibracion lista presiona un boton" + "\n")
		self.positions = positions
		self.list = []
		
		for i in range(len(self.positions)):
			self.list.append(Sensor(positions[i]))
			
	def getValues(self):
		values=[]
		for i in range(len(self.list)):
			values.append(self.list[i].getValue())
				
		return values
	
	def average(self):
		result = 0
		suma = 0
		for i in range(len(self.list)):
			result += i*1000*self.list[i].getValue()
			suma += self.list[i].getValue()
		return (result/suma)



