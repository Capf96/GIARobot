import serial
ser = serial.Serial('/dev/ttyACM0', 57600)
ser.flushInput()

class Arduino(object):

	def getQTR (self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(len(s)):
				s[i]=int(s[i])
		return s[0:8]
		
		
	def getUltraL(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[9])
		
		
	def getUltraR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[11])
	
	
	def getUltraC(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[10])
		
		
	def getEncoderL(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[12])
		
		
	def getEncoderL(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[13])
		
		
	def getAverageQTR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return int(s[8])
