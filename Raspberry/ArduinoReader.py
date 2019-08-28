import serial
ser = serial.Serial('/dev/ttyACM0', 57600)
ser.reset_input_buffer()

class Arduino(object):

	def getAll(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(len(s)):
				s[i]=float(s[i])
		return s

	def getQTR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(8):
				s[i]=int(s[i])
		return s[0:8]
		
	def getQTR2(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			for i in range(8):
				s[i]=int(s[i])
		return s[0:9]
		
		
	def getUltraL(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[9])
		
		
	def getUltraR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[11])
	
	
	def getUltraC(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[10])
		
		
	def getEncoderL(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return 360 - float(s[12])
		
		
	def getEncoderR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[13])
		
		
	def getAverageQTR(self):
		data = ('').join(list(ser.readline()))
		if data:
			result = str(ser.readline().strip())
			result = unicode(result, errors = 'replace' )
			s = result.decode('utf-8').split(' ')
			return float(s[8])
